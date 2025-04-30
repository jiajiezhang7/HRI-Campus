import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.action import ActionClient # type: ignore[attr-defined]
from rclpy.action.client import ClientGoalHandle, GoalStatus
from rclpy.duration import Duration as rclpyDuration
import yaml
import threading
import time
import math
import tf_transformations

class NavDialogueBridge(Node):
    def __init__(self):
        super().__init__('nav_dialogue_bridge')
        
        # 初始化导航器
        self.navigator = BasicNavigator()
        
        # 状态变量
        self.is_navigating = False
        self.target_location_keyword = None # 跟踪当前导航目标关键词
        self.current_goal_handle: ClientGoalHandle | None = None # 跟踪当前的 goal handle
        self._nav_lock = threading.Lock() # 确保导航启动的原子性

        # 订阅来自LLM的响应
        self.llm_sub = self.create_subscription(
            String,
            '/llm_response',
            self.llm_callback,
            10
        )
        
        # 创建发布者，用于发送导航完成后的响应消息
        self.response_pub = self.create_publisher(
            String,
            '/llm_response', # Use a different topic to avoid loops
            10
        )
        
        # 创建发布者，用于发送对话历史 (If needed, maybe rename topic)
        # self.history_pub = self.create_publisher(
        #     String,
        #     '/llm_conversation_history_navi', # Example rename
        #     10
        # )
        
        # 预设目标位置（会被配置文件覆盖）
        self.target_locations = {
            "kitchen": self.create_pose(1.0, 2.0, 0.0),
            "厨房": self.create_pose(1.0, 2.0, 0.0)
        }
        # 到达消息
        self.arrival_messages = {
            "kitchen": "我已经到达厨房。",
            "厨房": "我已经到达厨房。"
        }

        # 尝试加载配置文件
        self.declare_parameter('locations_config', '')
        config_file = self.get_parameter('locations_config').get_parameter_value().string_value
        if config_file:
            self.load_locations(config_file)
        else:
            self.get_logger().warn('No locations_config parameter specified. Using default locations.')

        # 添加初始位姿参数 (Ensure navigator uses it or set it manually)
        self.declare_parameter('initial_pose_x', 0.0)
        self.declare_parameter('initial_pose_y', 0.0)
        self.declare_parameter('initial_pose_theta', 0.0)
        
        # Wait for Nav2 services
        self.get_logger().info("Waiting for Nav2...")
        # You might need to adjust navigator names based on your setup
        # Example: navigator='bt_navigator', localizer='amcl'
        self.navigator.waitUntilNav2Active() 
        self.get_logger().info("Nav2 is active.")
        
        # Set initial pose if desired (Example)
        initial_x = self.get_parameter('initial_pose_x').get_parameter_value().double_value
        initial_y = self.get_parameter('initial_pose_y').get_parameter_value().double_value
        initial_theta = self.get_parameter('initial_pose_theta').get_parameter_value().double_value
        initial_pose = self.create_pose(initial_x, initial_y, initial_theta)
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info(f"Set initial pose to: x={initial_x}, y={initial_y}, theta={initial_theta}")

        # 订阅取消导航指令
        self.cancel_sub = self.create_subscription(
            String,
            '/cancel_navigation',
            self.cancel_navigation_callback,
            10
        )

        self.get_logger().info('NavDialogueBridge node started.')

    def load_locations(self, file_path):
        try:
            with open(file_path, 'r') as f:
                config = yaml.safe_load(f)
                if not config:
                    self.get_logger().warn(f"Config file {file_path} is empty.")
                    return

                loaded_locations = {}
                loaded_messages = {}
                for key, value in config.items():
                    if isinstance(value, dict) and 'pose' in value and 'arrival_message' in value:
                        pose_data = value['pose']
                        if isinstance(pose_data, list) and len(pose_data) == 3:
                            pose = self.create_pose(pose_data[0], pose_data[1], pose_data[2])
                            loaded_locations[key] = pose
                            loaded_messages[key] = value['arrival_message']
                            self.get_logger().info(f"Loaded location '{key}': Pose({pose_data}), Message: '{value['arrival_message']}'")
                        else:
                           self.get_logger().warn(f"Invalid pose format for key '{key}' in {file_path}. Expected [x, y, theta]. Skipping.")
                    else:
                         self.get_logger().warn(f"Invalid structure for key '{key}' in {file_path}. Expected 'pose' and 'arrival_message'. Skipping.")
                
                if loaded_locations:
                    self.target_locations = loaded_locations
                    self.arrival_messages = loaded_messages
                    self.get_logger().info(f"Successfully loaded locations and messages from {file_path}")
                else:
                     self.get_logger().warn(f"No valid locations found in {file_path}. Using defaults.")

        except FileNotFoundError:
            self.get_logger().error(f"Location configuration file not found: {file_path}")
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing YAML file {file_path}: {e}")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred while loading locations: {e}")


    def create_pose(self, x, y, theta_degrees):
        pose = PoseStamped()
        pose.header.frame_id = 'map'  # Or your relevant frame
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0  # Assuming 2D navigation

        # Convert degrees to radians for quaternion
        theta_rad = math.radians(theta_degrees)
        q = tf_transformations.quaternion_from_euler(0, 0, theta_rad)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def llm_callback(self, msg):
        response_text = msg.data.lower() # Convert to lower case for case-insensitive matching
        self.get_logger().info(f"Received LLM response: '{response_text}'")

        found_keyword = None
        for keyword in self.target_locations.keys():
             # Use lower case for comparison
            if keyword.lower() in response_text:
                found_keyword = keyword
                break # Take the first match

        if found_keyword:
            # Use a lock to prevent race conditions when checking/setting is_navigating
            with self._nav_lock:
                if self.is_navigating:
                    self.get_logger().warn(f"Navigation already in progress to '{self.target_location_keyword}'. Ignoring request for '{found_keyword}'.")
                    # Optionally publish a message indicating busy status
                    # self.response_pub.publish(String(data="我正在导航中，请稍后再试。"))
                    return

                # Set state *before* starting the thread/async call
                self.is_navigating = True
                self.target_location_keyword = found_keyword
                self.get_logger().info(f"Keyword '{found_keyword}' detected. Initiating navigation.")
                
                # --- Start Navigation Asynchronously ---
                target_pose = self.target_locations[found_keyword]
                self.navigator.clearTaskError() # Clear previous errors

                # Send the goal asynchronously, providing the feedback callback
                send_goal_future = self.navigator.nav_to_pose_client.send_goal_async(
                    NavigateToPose.Goal(pose=target_pose),
                    feedback_callback=self._nav_feedback_callback
                )
                
                # Add a callback to handle goal acceptance/rejection
                send_goal_future.add_done_callback(self._goal_accepted_callback)
        else:
            self.get_logger().debug(f"No navigation keywords found in '{response_text}'")


    def _goal_accepted_callback(self, future):
        """Callback executed when the goal server accepts/rejects the goal."""
        goal_handle = future.result()
        if not goal_handle:
            self.get_logger().error('Internal error getting goal handle')
            self.reset_navigation_state()
            return
            
        self.current_goal_handle = goal_handle # Store the handle

        if not goal_handle.accepted:
            self.get_logger().error(f"Goal for '{self.target_location_keyword}' was rejected by the server.")
            self.reset_navigation_state()
            # Publish failure message?
            self.response_pub.publish(String(data=f"无法启动导航至 {self.target_location_keyword}，目标被拒绝。"))
            return

        self.get_logger().info(f"Goal accepted for '{self.target_location_keyword}'. Navigation started.")
        
        # Goal accepted, now get the future for the final result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)


    def _nav_feedback_callback(self, feedback_msg):
        """Callback to receive navigation feedback."""
        feedback = feedback_msg.feedback
        # Log feedback periodically, maybe not every time to avoid spam
        # Example: Log distance remaining
        if feedback and hasattr(feedback, 'distance_remaining'):
             self.get_logger().debug(f"Feedback: Distance remaining: {feedback.distance_remaining:.2f}m")
        # Add other feedback processing here if needed
        # IMPORTANT: Don't publish arrival messages here, do it in the result callback


    def _get_result_callback(self, future):
        """Callback executed when the navigation action completes."""
        result_response = future.result()
        status = result_response.status
        result = result_response.result # The actual result message (e.g., NavigateToPose.Result)

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Navigation to '{self.target_location_keyword}' succeeded!")
            # Publish arrival message
            response_msg = self.arrival_messages.get(self.target_location_keyword)
            if response_msg:
                self.response_pub.publish(String(data=response_msg))
                self.get_logger().info(f"Published arrival message for {self.target_location_keyword}: '{response_msg}'")
            else:
                self.get_logger().warn(f"No arrival message defined for successful keyword: '{self.target_location_keyword}'")

        elif status == GoalStatus.STATUS_ABORTED:
            error_code = result.error_code if hasattr(result, 'error_code') else 'N/A'
            error_msg = result.error_msg if hasattr(result, 'error_msg') else 'No message'
            self.get_logger().error(f"Navigation to '{self.target_location_keyword}' failed (Aborted). Code: {error_code}, Msg: {error_msg}")
            self.response_pub.publish(String(data=f"导航至 {self.target_location_keyword} 失败了。"))

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f"Navigation to '{self.target_location_keyword}' was canceled.")
            # Optional: Publish cancellation confirmation
            self.response_pub.publish(String(data=f"已取消导航至 {self.target_location_keyword}。"))

        else:
            self.get_logger().warn(f"Navigation to '{self.target_location_keyword}' finished with unknown status: {status}")
            self.response_pub.publish(String(data=f"导航至 {self.target_location_keyword} 状态未知。"))

        # Reset navigation state regardless of outcome
        self.reset_navigation_state()


    def cancel_navigation_callback(self, msg):
        """Callback to handle external cancellation requests."""
        command = msg.data.lower()
        if command == "cancel":
            self.get_logger().info("Received cancellation request.")
            self.cancel_current_navigation()
        else:
            self.get_logger().warn(f"Received unknown command on /cancel_navigation: {command}")


    def cancel_current_navigation(self):
         """Initiates cancellation of the current navigation task."""
         with self._nav_lock:
            if self.is_navigating and self.current_goal_handle:
                self.get_logger().info(f"Attempting to cancel navigation to '{self.target_location_keyword}'...")
                cancel_future = self.current_goal_handle.cancel_goal_async()
                # Add a callback to confirm cancellation (optional but good practice)
                cancel_future.add_done_callback(self._cancel_done_callback)
            elif self.is_navigating:
                 self.get_logger().warn("Cancellation requested, but goal handle is missing.")
                 # Reset state anyway?
                 self.reset_navigation_state() 
            else:
                self.get_logger().info("No active navigation to cancel.")


    def _cancel_done_callback(self, future):
        """Callback executed when the cancellation request is processed."""
        cancel_response = future.result()
        if cancel_response:
             if len(cancel_response.goals_canceling) > 0:
                 self.get_logger().info("Cancellation request accepted.")
                 # The final status (CANCELED) will be handled by _get_result_callback
             else:
                 self.get_logger().warn("Cancellation request processed, but no goals were marked for cancellation (task might have already finished).")
                 # If the task finished before cancellation was processed, reset state might be needed here
                 # depending on whether _get_result_callback was already called.
                 # Check self.is_navigating here if necessary.
        else:
            self.get_logger().error("Failed to process cancellation request.")
            # Consider resetting state here too as a fallback
            self.reset_navigation_state()


    def reset_navigation_state(self):
        """Resets the navigation-related state variables."""
        with self._nav_lock: # Ensure atomicity
            self.is_navigating = False
            self.target_location_keyword = None
            self.current_goal_handle = None
        self.get_logger().debug("Navigation state reset.")


def main(args=None):
    rclpy.init(args=args)
    node = NavDialogueBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down.')
    finally:
        # Clean up resources
        node.navigator.destroyNode() # Use the navigator's cleanup
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()