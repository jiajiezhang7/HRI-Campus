#include <stdio.h>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <boost/thread.hpp>
#include <vector>
#include <cmath>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "audio_common_msgs/msg/audio_data.hpp"
#include "audio_common_msgs/msg/audio_data_stamped.hpp"
#include "audio_common_msgs/msg/audio_info.hpp"
#include "std_msgs/msg/bool.hpp"

namespace audio_capture
{
  // 高通滤波器类
  class HighPassFilter {
  private:
    double cutoffFrequency;  // 截止频率
    double sampleRate;       // 采样率
    double alpha;            // 滤波器系数
    std::vector<float> xv;   // 输入历史
    std::vector<float> yv;   // 输出历史

  public:
    HighPassFilter(double cutoffFreq, double sampRate) 
      : cutoffFrequency(cutoffFreq), sampleRate(sampRate) {
      // 计算滤波器系数
      double dt = 1.0 / sampleRate;
      double RC = 1.0 / (2.0 * M_PI * cutoffFrequency);
      alpha = RC / (RC + dt);
      
      // 初始化历史数据
      xv.resize(2, 0.0);
      yv.resize(2, 0.0);
    }

    // 处理单个样本
    float process(float sample) {
      // 移动历史数据
      xv[0] = xv[1];
      yv[0] = yv[1];
      
      // 新输入
      xv[1] = sample;
      
      // 高通滤波器公式: y[n] = alpha * (y[n-1] + x[n] - x[n-1])
      yv[1] = alpha * (yv[0] + xv[1] - xv[0]);
      
      return yv[1];
    }

    // 处理一块PCM数据
    void processBlock(int16_t* buffer, size_t size) {
      for (size_t i = 0; i < size; i++) {
        float sample = static_cast<float>(buffer[i]);
        float filtered = process(sample);
        buffer[i] = static_cast<int16_t>(filtered);
      }
    }
  };

  class AudioCaptureFilterNode: public rclcpp::Node
  {
    public:
      AudioCaptureFilterNode(const rclcpp::NodeOptions & options)
      : Node("audio_capture_filter_node", options), updater_(this), _desired_rate(-1.0), _muted(false)
      {
        gst_init(nullptr, nullptr);

        _bitrate = 192;
        std::string src_type;
        std::string dst_type;
        std::string device;

        // Need to encoding or publish raw wave data
        this->declare_parameter<std::string>("format", "mp3");
        this->declare_parameter<std::string>("sample_format", "S16LE");
        this->get_parameter("format", _format);
        this->get_parameter("sample_format", _sample_format);

        // The bitrate at which to encode the audio
        this->declare_parameter<int>("bitrate", 192);
        this->get_parameter("bitrate", _bitrate);

        // only available for raw data
        this->declare_parameter<int>("channels", 1);
        this->declare_parameter<int>("depth", 16);
        this->declare_parameter<int>("sample_rate", 16000);
        this->get_parameter("channels", _channels);
        this->get_parameter("depth", _depth);
        this->get_parameter("sample_rate", _sample_rate);

        // 高通滤波器参数
        this->declare_parameter<bool>("enable_filter", false);
        this->declare_parameter<double>("cutoff_frequency", 100.0);
        this->get_parameter("enable_filter", _enable_filter);
        this->get_parameter("cutoff_frequency", _cutoff_frequency);

        if (_enable_filter) {
          RCLCPP_INFO(this->get_logger(), "高通滤波器已启用，截止频率：%.1f Hz", _cutoff_frequency);
          _filter_processor = std::make_unique<HighPassFilter>(_cutoff_frequency, _sample_rate);
        } else {
          RCLCPP_INFO(this->get_logger(), "高通滤波器未启用");
        }

        // The destination of the audio
        this->declare_parameter<std::string>("dst", "appsink");
        this->get_parameter("dst", dst_type);

        // The source of the audio
        this->declare_parameter<std::string>("src", "alsasrc");
        this->get_parameter("src", src_type);
        this->declare_parameter<std::string>("device", "");
        this->get_parameter("device", device);
        
        // 添加GStreamer音频捕获参数
        this->declare_parameter<int64_t>("buffer_time", 100000);  // 默认100毫秒
        this->declare_parameter<int64_t>("latency_time", 5000);   // 默认5毫秒
        int64_t buffer_time, latency_time;
        this->get_parameter("buffer_time", buffer_time);
        this->get_parameter("latency_time", latency_time);

        _pub = this->create_publisher<audio_common_msgs::msg::AudioData>("audio", 10);
        auto info_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        _pub_info = this->create_publisher<audio_common_msgs::msg::AudioInfo>("audio_info", info_qos);

        rclcpp::Publisher<audio_common_msgs::msg::AudioDataStamped>::SharedPtr pub_stamped =
          this->create_publisher<audio_common_msgs::msg::AudioDataStamped>("audio_stamped", 10);

        // 创建订阅者，订阅麦克风静音状态
        _mute_subscription = this->create_subscription<std_msgs::msg::Bool>(
          "/mic_mute",
          1,  // 高优先级
          std::bind(&AudioCaptureFilterNode::mute_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "已创建 /mic_mute 订阅，将根据静音信号控制音频发布");

        this->declare_parameter<double>("diagnostic_tolerance", 0.1);
        auto tolerance = this->get_parameter("diagnostic_tolerance").as_double();

        updater_.setHardwareID("microphone");
        _diagnosed_pub_stamped =
          std::make_shared<diagnostic_updater::DiagnosedPublisher<audio_common_msgs::msg::AudioDataStamped>>(
            pub_stamped, updater_,
            diagnostic_updater::FrequencyStatusParam(&_desired_rate, &_desired_rate, tolerance, 10),
            diagnostic_updater::TimeStampStatusParam());

        _loop = g_main_loop_new(NULL, false);
        _pipeline = gst_pipeline_new("ros_pipeline");
        GstClock *clock = gst_system_clock_obtain();
        g_object_set(clock, "clock-type", GST_CLOCK_TYPE_REALTIME, NULL);
        gst_pipeline_use_clock(GST_PIPELINE_CAST(_pipeline), clock);
        gst_object_unref(clock);

        _bus = gst_pipeline_get_bus(GST_PIPELINE(_pipeline));
        gst_bus_add_signal_watch(_bus);
        g_signal_connect(_bus, "message::error",
                         G_CALLBACK(onMessage), this);
        g_object_unref(_bus);

        // We create the sink first, just for convenience
        if (dst_type == "appsink")
        {
          _sink = gst_element_factory_make("appsink", "sink");
          g_object_set(G_OBJECT(_sink), "emit-signals", true, NULL);
          g_object_set(G_OBJECT(_sink), "max-buffers", 100, NULL);
          g_signal_connect( G_OBJECT(_sink), "new-sample",
                            G_CALLBACK(onNewBuffer), this);
        }
        else
        {
          RCLCPP_INFO_STREAM(this->get_logger(), "file sink to " << dst_type.c_str());
          _sink = gst_element_factory_make("filesink", "sink");
          g_object_set( G_OBJECT(_sink), "location", dst_type.c_str(), NULL);
        }

        _source = gst_element_factory_make(src_type.c_str(), "source");
        // if device isn't specified, it will use the default which is
        // the alsa default source.
        // A valid device will be of the foram hw:0,0 with other numbers
        // than 0 and 0 as are available.
        if (device != "")
        {
          // ghcar *gst_device = device.c_str();
          g_object_set(G_OBJECT(_source), "device", device.c_str(), NULL);
        }
        
        // 设置音频捕获参数以提高灵敏度
        g_object_set(G_OBJECT(_source), "buffer-time", buffer_time, NULL);
        g_object_set(G_OBJECT(_source), "latency-time", latency_time, NULL);
        RCLCPP_INFO(this->get_logger(), "设置音频捕获参数: buffer-time=%ld μs, latency-time=%ld μs", 
                   buffer_time, latency_time);

        GstCaps *caps;
        caps = gst_caps_new_simple("audio/x-raw",
                                   "format", G_TYPE_STRING, _sample_format.c_str(),
                                   "channels", G_TYPE_INT, _channels,
                                   "width",    G_TYPE_INT, _depth,
                                   "depth",    G_TYPE_INT, _depth,
                                   "rate",     G_TYPE_INT, _sample_rate,
                                   "signed",   G_TYPE_BOOLEAN, TRUE,
                                   NULL);

        gboolean link_ok;
        if (_format == "mp3"){
          _filter = gst_element_factory_make("capsfilter", "filter");
          g_object_set( G_OBJECT(_filter), "caps", caps, NULL);
          gst_caps_unref(caps);

          _convert = gst_element_factory_make("audioconvert", "convert");
          if (!_convert) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to create audioconvert element");
            exitOnMainThread(1);
          }

          _encode = gst_element_factory_make("lamemp3enc", "encoder");
          if (!_encode) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to create encoder element");
            exitOnMainThread(1);
          }
          g_object_set( G_OBJECT(_encode), "target", 1, NULL);
          g_object_set( G_OBJECT(_encode), "bitrate", _bitrate, NULL);

          gst_bin_add_many( GST_BIN(_pipeline), _source, _filter, _convert, _encode, _sink, NULL);
          link_ok = gst_element_link_many(_source, _filter, _convert, _encode, _sink, NULL);
        } else if (_format == "wave") {
          if (dst_type == "appsink") {
            g_object_set( G_OBJECT(_sink), "caps", caps, NULL);
            gst_caps_unref(caps);
            gst_bin_add_many( GST_BIN(_pipeline), _source, _sink, NULL);
            link_ok = gst_element_link_many( _source, _sink, NULL);
          } else {
            _filter = gst_element_factory_make("wavenc", "filter");
            gst_bin_add_many( GST_BIN(_pipeline), _source, _filter, _sink, NULL);
            link_ok = gst_element_link_many( _source, _filter, _sink, NULL);
          }
        } else {
          RCLCPP_ERROR_STREAM(this->get_logger(), "format must be \"wave\" or \"mp3\"");
          exitOnMainThread(1);
        }

        if (!link_ok) {
          RCLCPP_ERROR_STREAM(this->get_logger(), "Unsupported media type.");
          exitOnMainThread(1);
        }

        gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING);

        _gst_thread = boost::thread( boost::bind(g_main_loop_run, _loop) );

        _timer_info = rclcpp::create_timer(this, get_clock(), std::chrono::seconds(5), [this] { publishInfo(); });
        publishInfo();
      }

      void publishInfo() {
        audio_common_msgs::msg::AudioInfo info_msg;
        info_msg.channels = _channels;
        info_msg.sample_rate = _sample_rate;
        info_msg.sample_format = _sample_format;
        info_msg.bitrate = _bitrate;
        info_msg.coding_format = _format;
        _pub_info->publish(info_msg);
      }

      ~AudioCaptureFilterNode()
      {
        g_main_loop_quit(_loop);
        gst_element_set_state(_pipeline, GST_STATE_NULL);
        gst_object_unref(_pipeline);
        g_main_loop_unref(_loop);
      }

      // 处理麦克风静音控制信号的回调函数
      void mute_callback(const std_msgs::msg::Bool::SharedPtr msg)
      {
        if (msg->data && !_muted) {
          _muted = true;
          RCLCPP_INFO(this->get_logger(), "收到静音信号，暂停发布音频数据");
        } else if (!msg->data && _muted) {
          _muted = false;
          RCLCPP_INFO(this->get_logger(), "收到取消静音信号，恢复发布音频数据");
        }
      }

    private:
      diagnostic_updater::Updater updater_;
      std::shared_ptr<diagnostic_updater::DiagnosedPublisher<audio_common_msgs::msg::AudioDataStamped>> _diagnosed_pub_stamped;
      double _desired_rate;

      rclcpp::Publisher<audio_common_msgs::msg::AudioData>::SharedPtr _pub;
      rclcpp::Publisher<audio_common_msgs::msg::AudioInfo>::SharedPtr _pub_info;
      rclcpp::TimerBase::SharedPtr _timer_info;
      rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _mute_subscription;
      bool _muted;

      // GStreamer 管道和元素
      GstElement *_pipeline, *_source, *_filter, *_sink, *_convert, *_encode;
      GstBus *_bus;
      GMainLoop *_loop;
      boost::thread _gst_thread;

      // 音频参数
      std::string _format;
      std::string _sample_format;
      int _channels;
      int _depth;
      int _sample_rate;
      int _bitrate;

      // 高通滤波器参数
      bool _enable_filter;
      double _cutoff_frequency;
      std::unique_ptr<HighPassFilter> _filter_processor;

      // 处理新的音频缓冲区
      static GstFlowReturn onNewBuffer(GstAppSink *appsink, gpointer userData) {
        AudioCaptureFilterNode *node = static_cast<AudioCaptureFilterNode*>(userData);
        GstSample *sample = gst_app_sink_pull_sample(appsink);
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        GstMapInfo map;
        
        if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
          // 检查麦克风是否处于静音状态
          if (node->_muted) {
            // 麦克风静音，不发布音频数据
            gst_buffer_unmap(buffer, &map);
            gst_sample_unref(sample);
            return GST_FLOW_OK;
          }
          
          audio_common_msgs::msg::AudioData msg;
          
          // 如果启用了滤波器并且是wave格式，应用高通滤波器
          if (node->_enable_filter && node->_format == "wave") {
            // 创建一个副本以便修改
            std::vector<uint8_t> filtered_data(map.data, map.data + map.size);
            
            // 应用高通滤波器
            if (node->_sample_format == "S16LE") {
              // 假设16位有符号小端格式
              int16_t* pcm_data = reinterpret_cast<int16_t*>(filtered_data.data());
              size_t sample_count = filtered_data.size() / sizeof(int16_t);
              node->_filter_processor->processBlock(pcm_data, sample_count);
            }
            
            // 发布过滤后的数据
            msg.data = std::vector<uint8_t>(filtered_data.begin(), filtered_data.end());
          } else {
            // 直接发布原始数据
            msg.data = std::vector<uint8_t>(map.data, map.data + map.size);
          }
          
          node->_pub->publish(msg);
          
          // 创建带时间戳的消息
          audio_common_msgs::msg::AudioDataStamped stamped_msg;
          stamped_msg.header.stamp = node->now();
          stamped_msg.audio = msg;
          node->_diagnosed_pub_stamped->publish(stamped_msg);
          
          gst_buffer_unmap(buffer, &map);
        }
        
        gst_sample_unref(sample);
        return GST_FLOW_OK;
      }

      // 处理GStreamer消息
      static gboolean onMessage(GstBus *bus, GstMessage *message, gpointer userData) {
        AudioCaptureFilterNode *node = static_cast<AudioCaptureFilterNode*>(userData);
        GError *err;
        gchar *debug;
        
        gst_message_parse_error(message, &err, &debug);
        RCLCPP_ERROR(node->get_logger(), "GStreamer Pipeline Error: %s", err->message);
        g_error_free(err);
        g_free(debug);
        
        g_main_loop_quit(node->_loop);
        return FALSE;
      }

      void exitOnMainThread(int code) {
        exit(code);
      }
  };
}

RCLCPP_COMPONENTS_REGISTER_NODE(audio_capture::AudioCaptureFilterNode)
