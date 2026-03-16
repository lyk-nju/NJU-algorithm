#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import serial
import threading
import os
import time
import numpy as np

# ================= 配置区域 =================
SERIAL_PORT = '/dev/ttyACM1'   # 下位机串口
BAUD_RATE = 115200
IMAGE_TOPIC = '/image_raw'     # 订阅的图像话题
SAVE_FOLDER = "assets/img_with_q"
# ===========================================

class ImageSubscriber(Node):
    """
    ROS2 图像接收节点，对应你C++里的 ROS2Manager
    """
    def __init__(self):
        super().__init__('data_collector_node')
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_mutex = threading.Lock()
        
        # 创建订阅
        self.subscription = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.image_callback,
            10
        )
        self.get_logger().info(f'已订阅话题: {IMAGE_TOPIC}')

    def image_callback(self, msg):
        try:
            # 将 ROS 图像消息转换为 OpenCV 格式 (BGR8)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            with self.frame_mutex:
                self.latest_frame = cv_image
                
        except Exception as e:
            self.get_logger().error(f'cv_bridge转换错误: {e}')

    def get_latest_frame(self):
        """线程安全地获取当前帧"""
        with self.frame_mutex:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()

class SerialReader:
    """
    串口读取类，负责解析 w,x,y,z,yaw,pitch
    """
    def __init__(self):
        self.ser = None
        self.latest_data = None # [w, x, y, z, yaw, pitch]
        self.latest_raw_str = None
        self.running = True
        
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print(f"[Serial] 成功连接: {SERIAL_PORT}")
        except Exception as e:
            print(f"[Serial] 连接失败: {e}")

    def start(self):
        t = threading.Thread(target=self._read_loop)
        t.daemon = True
        t.start()

    def _read_loop(self):
        if not self.ser: return
        
        while self.running:
            try:
                if self.ser.in_waiting:
                    # 读取并解码，忽略解码错误
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    # 简单校验: "w,x,y,z,yaw,pitch" 应该有5个逗号
                    if line.count(',') == 5:
                        parts = line.split(',')
                        try:
                            # 解析数值 [w, x, y, z, yaw, pitch]
                            values = [float(x) for x in parts]
                            self.latest_data = values
                            self.latest_raw_str = line
                        except ValueError:
                            pass # 解析数字失败，跳过
            except Exception as e:
                print(f"[Serial] 读取错误: {e}")
                time.sleep(0.1)

    def stop(self):
        self.running = False
        if self.ser:
            self.ser.close()

def main():
    # 1. 初始化保存目录
    if not os.path.exists(SAVE_FOLDER):
        os.makedirs(SAVE_FOLDER)

    # 2. 初始化 ROS2
    rclpy.init()
    ros_node = ImageSubscriber()
    
    # 在独立线程中运行 ROS spin，这样不会阻塞 cv2.imshow
    ros_thread = threading.Thread(target=lambda: rclpy.spin(ros_node))
    ros_thread.daemon = True
    ros_thread.start()

    # 3. 初始化串口读取
    serial_reader = SerialReader()
    serial_reader.start()

    print("=============================================")
    print("   ROS2 手眼标定数据采集工具启动")
    print(f"   图像话题: {IMAGE_TOPIC}")
    print(f"   串口设备: {SERIAL_PORT}")
    print("   按 'S' 保存当前帧和位姿")
    print("   按 'Q' 退出")
    print("=============================================")

    img_count = 1
    # 避免覆盖已有文件
    while os.path.exists(f"{SAVE_FOLDER}/{img_count}.jpg"):
        img_count += 1

    # 4. 主循环 (UI显示与交互)
    try:
        while True:
            # 获取数据
            frame = ros_node.get_latest_frame()
            pose_data = serial_reader.latest_data
            
            if frame is None:
                # 如果还没收到图像，显示一个等待画面
                wait_img = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(wait_img, "Waiting for ROS Image...", (50, 240), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                cv2.imshow("Data Collection", wait_img)
            else:
                display_img = frame.copy()
                
                # 在画面上显示状态
                if pose_data:
                    # pose_data: [w, x, y, z, yaw, pitch]
                    info_color = (0, 255, 0)
                    text = f"IMU: Yaw={pose_data[4]:.2f} Pitch={pose_data[5]:.2f}"
                else:
                    info_color = (0, 0, 255)
                    text = "IMU: No Data"
                
                cv2.putText(display_img, text, (20, 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, info_color, 2)
                
                cv2.putText(display_img, f"Count: {img_count}", (20, 80), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

                cv2.imshow("Data Collection", display_img)

            # 按键处理
            key = cv2.waitKey(30) & 0xFF
            
            if key == ord('q'):
                break
            
            elif key == ord('s'):
                if frame is not None and pose_data is not None:
                    # 保存图片
                    cv2.imwrite(f"{SAVE_FOLDER}/{img_count}.jpg", frame)
                    
                    # 保存位姿 (w x y z) -> 兼容之前的读取逻辑
                    # 你的C++发送顺序是 w,x,y,z，我们直接存这4个
                    w, x, y, z = pose_data[0], pose_data[1], pose_data[2], pose_data[3]
                    with open(f"{SAVE_FOLDER}/{img_count}.txt", "w") as f:
                        f.write(f"{w} {x} {y} {z}")
                    
                    print(f"[已保存] {img_count}.jpg | Yaw:{pose_data[4]:.2f}")
                    img_count += 1
                else:
                    print("[警告] 数据不全 (缺少图像或IMU)，无法保存！")

    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        serial_reader.stop()
        ros_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
