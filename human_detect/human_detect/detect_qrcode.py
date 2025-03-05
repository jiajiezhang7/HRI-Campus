import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from facenet_pytorch import MTCNN
from PIL import Image as PILImage
import os

class PanoramaHumanDetector(Node):
    def __init__(self):
        # 初始化ROS2节点
        super().__init__('panorama_human_detector')
        self.bridge = CvBridge()

        # 订阅图像话题
        self.image_sub = self.create_subscription(
            Image, '/insta360_air/image_get', self.image_callback, 1)

        # 初始化YOLOv8，只检测"person"
        self.yolo = YOLO('yolov8s.pt', verbose=False)  # 使用YOLOv8 small模型
        self.yolo.classes = [0]  # YOLOv8的'person'类编号为0

        self.mtcnn = MTCNN(keep_all=True)  # 保持检测到的所有人脸
        # self.save_path = '/home/agilex03/agilex_ws/src/human_detect/detected_images/'  # 图片保存目录
        self.counter = 0  # 图片命名计数器
        # if not os.path.exists(self.save_path):
        #     os.makedirs(self.save_path)

        # 发布器，发布面向相机的人脸角度
        self.face_angle_pub = self.create_publisher(Float32, '/face_angle', 1)

        # 每两秒触发一次回调
        # self.timer = self.create_timer(2.0, self.image_callback)
        qrcode = cv2.QRCodeDetector()

    def eqruirect2persp(self, img, FOV, THETA, PHI, Hd, Wd):
        equ_h, equ_w = img.shape[:2]

        equ_cx = (equ_w) / 2.0
        equ_cy = (equ_h) / 2.0

        wFOV = FOV
        hFOV = float(Hd) / Wd * wFOV

        c_x = (Wd) / 2.0
        c_y = (Hd) / 2.0

        w_len = 2 * np.tan(np.radians(wFOV / 2.0))
        w_interval = w_len / (Wd)

        h_len = 2 * np.tan(np.radians(hFOV / 2.0))
        h_interval = h_len / (Hd)

        x_map = np.zeros([Hd, Wd], np.float32) + 1
        y_map = np.tile((np.arange(0, Wd) - c_x) * w_interval, [Hd, 1])
        z_map = -np.tile((np.arange(0, Hd) - c_y) * h_interval, [Wd, 1]).T
        D = np.sqrt(x_map ** 2 + y_map ** 2 + z_map ** 2)

        xyz = np.zeros([Hd, Wd, 3], float)
        xyz[:, :, 0] = (x_map / D)[:, :]
        xyz[:, :, 1] = (y_map / D)[:, :]
        xyz[:, :, 2] = (z_map / D)[:, :]

        y_axis = np.array([0.0, 1.0, 0.0], np.float32)
        z_axis = np.array([0.0, 0.0, 1.0], np.float32)
        [R1, _] = cv2.Rodrigues(z_axis * np.radians(THETA))
        [R2, _] = cv2.Rodrigues(np.dot(R1, y_axis) * np.radians(-PHI))

        xyz = xyz.reshape([Hd * Wd, 3]).T
        xyz = np.dot(R1, xyz)
        xyz = np.dot(R2, xyz).T
        lat = np.arcsin(xyz[:, 2] / 1)
        lon = np.zeros([Hd * Wd], float)
        theta = np.arctan(xyz[:, 1] / xyz[:, 0])
        idx1 = xyz[:, 0] > 0
        idx2 = xyz[:, 1] > 0

        idx3 = ((1 - idx1) * idx2).astype(bool)
        idx4 = ((1 - idx1) * (1 - idx2)).astype(bool)

        lon[idx1] = theta[idx1]
        lon[idx3] = theta[idx3] + np.pi
        lon[idx4] = theta[idx4] - np.pi

        lon = lon.reshape([Hd, Wd]) / np.pi * 180
        lat = -lat.reshape([Hd, Wd]) / np.pi * 180
        lon = lon / 180 * equ_cx + equ_cx
        lat = lat / 90 * equ_cy + equ_cy

        persp = cv2.remap(img,
                          lon.astype(np.float32),
                          lat.astype(np.float32),
                          cv2.INTER_CUBIC,
                          borderMode=cv2.BORDER_WRAP)

        return persp

    def image_callback(self, msg):
        # 从ROS消息中获取图像
        if msg is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # 图像分成四部分
        Wd, Hd = 640, 640  # 设定裁剪图像的宽高
        images = [
            self.eqruirect2persp(cv_image, 90, 0, 20, Hd, Wd),      # 0° - 90°
            self.eqruirect2persp(cv_image, 90, 90, 20, Hd, Wd),     # 90° - 180°
            self.eqruirect2persp(cv_image, 90, 180, 20, Hd, Wd),    # 180° - 270°
            self.eqruirect2persp(cv_image, 90, 270, 20, Hd, Wd)     # 270° - 360°
        ]

        closest_face_angle = None
        closest_distance = float('inf')  # 初始化为最大值

        # 为同一帧的四张图保持一致的编号
        frame_id = str(self.counter).zfill(4)
        self.counter = (self.counter + 1) % 10000  # 超过9999则重新计数

        # 遍历每个裁剪的图像，进行人脸检测与姿态判断
        for idx, img in enumerate(images):
            value, pts, qr_code = qrcode(img)
            if value is not None:  # 如果检测到QR码
                # 画出QR码的边框
                pts = pts[0].astype(int)
                for i in range(4):
                    cv2.line(img, tuple(pts[i]), tuple(pts[(i + 1) % 4]), (0, 255, 0), 2)

        # 在QR码框附近绘制文本
                cv2.putText(img, f'QR Code: {value}', (pts[0][0], pts[0][1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # 可选：保存带有QR码框的图片
        # self.save_detected_image(img, frame_id, idx + 1)

    # 保存带有检测框和标签的图片
        # self.save_detected_image(img, frame_id, idx + 1)


    def calculate_face_angle(self, face_center, img_idx):
        """计算人脸在全景图中的角度 (0-360)"""
        angle_ranges = [(0, 90), (90, 180), (180, 270), (270, 360)]
        img_angle_start = angle_ranges[img_idx][0]
        img_angle_range = angle_ranges[img_idx][1] - img_angle_start

        # 假设图像的宽度为 640 像素，根据人脸中心的横坐标计算角度
        angle = img_angle_start + (face_center[0] / 640.0) * img_angle_range
        return angle

    def is_facing_camera(self, face_center, person_center):
        """判断人脸是否面向相机，假设面向相机时人脸中心靠近人的中心"""
        return abs(face_center[0] - person_center[0]) < 100

    def get_person_center(self, box):
        """计算检测框的中心点"""
        x1, y1, x2, y2 = box
        return ((x1 + x2) / 2, (y1 + y2) / 2)

    def detect_faces(self, image):
        """使用MTCNN进行人脸检测"""
        pil_image = PILImage.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

        ## 调试bug 为什么检测会down 图片size太小？？
        # self.get_logger().info(f"Image size: {pil_image.size}")

        # 检查图像尺寸，跳过非常小的图像
        if pil_image.width < 20 or pil_image.height < 20:
            self.get_logger().info(f"图像尺寸太小，跳过人脸检测: {pil_image.size}")
            return []
            
        boxes, probs = self.mtcnn.detect(pil_image)
        return boxes if boxes is not None else []

    def save_detected_image(self, img, frame_id, img_part_idx):
        # 命名方式 ****_1, ****_2, ****_3, ****_4
        # file_name = f"{self.save_path}{frame_id}_{img_part_idx}.jpg"
        # cv2.imwrite(file_name, img)
        # self.get_logger().info(f"保存检测图像: {file_name}")
        pass

    def draw_box_with_label(self, img, box, label, color=(255, 0, 0), thickness=2):
        """在图像上绘制边界框和标签"""
        x1, y1, x2, y2 = map(int, box)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)
        # 在框的上方绘制标签
        label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
        label_y = max(y1 - 5, 0)
        cv2.rectangle(img, (x1, label_y - label_size[1]), (x1 + label_size[0], label_y), color, cv2.FILLED)
        cv2.putText(img, label, (x1, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)


def main(args=None):
    rclpy.init(args=args)
    node = PanoramaHumanDetector()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()