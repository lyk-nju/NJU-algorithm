#!/usr/bin/env python3
"""
相机内参标定节点
从ROS2的image_raw话题读取图像，使用12x7的棋盘格标定板进行标定
每个棋盘格的宽度是10mm
按C键采集图片，按Q键完成采集并进行标定
"""

import os
import sys

# 必须在导入OpenCV之前设置环境变量
# 强制使用GTK后端，避免Qt相关的段错误
os.environ['QT_QPA_PLATFORM'] = 'offscreen'
# 禁用OpenCV的并行处理，可能与ROS2冲突
os.environ['OPENCV_VIDEOIO_PRIORITY_MSMF'] = '0'

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from datetime import datetime

# 禁用OpenCV多线程，避免与ROS2冲突
cv2.setNumThreads(1)


class CameraCalibrationNode(Node):
    def __init__(self):
        super().__init__('camera_calibration_node')
        
        # 声明参数
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('chessboard_width', 12)  # 棋盘格内角点列数
        self.declare_parameter('chessboard_height', 7)  # 棋盘格内角点行数
        self.declare_parameter('square_size', 10.0)     # 每个棋盘格的宽度(mm)
        self.declare_parameter('min_images', 15)        # 最少需要采集的图片数量
        self.declare_parameter('use_rational_model', False)  # 是否使用有理模型（包含k3）
        
        # 获取参数
        self.image_topic = self.get_parameter('image_topic').value
        self.chessboard_size = (
            self.get_parameter('chessboard_width').value,
            self.get_parameter('chessboard_height').value
        )
        self.square_size = self.get_parameter('square_size').value
        self.min_images = self.get_parameter('min_images').value
        self.use_rational_model = self.get_parameter('use_rational_model').value
        
        # 初始化
        self.bridge = CvBridge()
        self.current_frame = None
        self.captured_images = []
        self.captured_corners = []
        
        # GUI状态
        self.capture_requested = False
        self.quit_requested = False
        self.button_area_capture = None  # 采集按钮区域
        self.button_area_quit = None     # 退出按钮区域
        self.last_ret = False
        self.last_corners = None
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        
        # 准备棋盘格的3D点
        self.objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 
                                     0:self.chessboard_size[1]].T.reshape(-1, 2)
        self.objp *= self.square_size
        
        # 创建保存目录
        self.save_dir = os.path.join(os.path.dirname(__file__), 'calibration_images')
        os.makedirs(self.save_dir, exist_ok=True)
        
        self.get_logger().info(f'相机标定节点已启动')
        self.get_logger().info(f'订阅话题: {self.image_topic}')
        self.get_logger().info(f'棋盘格尺寸: {self.chessboard_size[0]}x{self.chessboard_size[1]}')
        self.get_logger().info(f'棋盘格大小: {self.square_size}mm')
        self.get_logger().info(f'畸变模型: {"有理模型(k1-k3)" if self.use_rational_model else "标准模型(k1-k2)"}')
        self.get_logger().info(f'点击界面上的按钮进行操作：[采集图片] 或 [完成标定]')
        
    def image_callback(self, msg):
        """接收图像的回调函数"""
        try:
            # 不指定encoding，让cv_bridge使用默认转换
            # 然后手动转换颜色空间以避免cv_bridge的cvtColor冲突
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # 如果是mono8，转换为BGR
            if len(frame.shape) == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            # 如果是RGB，转换为BGR
            elif frame.shape[2] == 3 and msg.encoding == 'rgb8':
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            # 如果是RGBA，转换为BGR
            elif frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            
            # 验证图像数据
            if frame is not None and frame.size > 0:
                self.current_frame = frame
            else:
                self.get_logger().warning('接收到无效的图像数据')
        except Exception as e:
            self.get_logger().error(f'转换图像失败: {str(e)}')
            self.get_logger().error(f'图像编码: {msg.encoding}, 尺寸: {msg.width}x{msg.height}')
            import traceback
            traceback.print_exc()
    
    def find_and_draw_corners(self, image):
        """查找并绘制棋盘格角点"""
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # 尝试多种检测策略
            ret = False
            corners = None
            
            # 策略1: 标准检测（快速）
            ret, corners = cv2.findChessboardCorners(
                gray, 
                self.chessboard_size,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
            )
            
            # 策略2: 如果失败，尝试不使用FAST_CHECK（更准确但慢）
            if not ret:
                ret, corners = cv2.findChessboardCorners(
                    gray, 
                    self.chessboard_size,
                    cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
                )
            
            # 策略3: 如果还是失败，尝试增强对比度后检测
            if not ret:
                # 应用直方图均衡化增强对比度
                gray_enhanced = cv2.equalizeHist(gray)
                ret, corners = cv2.findChessboardCorners(
                    gray_enhanced, 
                    self.chessboard_size,
                    cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
                )
            
            # 策略4: 尝试高斯模糊后检测（降噪）
            if not ret:
                gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)
                ret, corners = cv2.findChessboardCorners(
                    gray_blur, 
                    self.chessboard_size,
                    cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
                )
                
        except Exception as e:
            self.get_logger().error(f'查找角点时出错: {str(e)}')
            return False, image.copy(), None
        
        display_image = image.copy()
        
        if ret:
            # 亚像素角点优化
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            # 绘制角点
            cv2.drawChessboardCorners(display_image, self.chessboard_size, corners_refined, ret)
            
            # 添加绿色边框表示检测成功
            cv2.rectangle(display_image, (10, 10), 
                         (display_image.shape[1]-10, display_image.shape[0]-10), 
                         (0, 255, 0), 3)
            
            return True, display_image, corners_refined
        else:
            # 添加红色边框表示未检测到
            cv2.rectangle(display_image, (10, 10), 
                         (display_image.shape[1]-10, display_image.shape[0]-10), 
                         (0, 0, 255), 3)
            
            # 添加提示信息
            hint_text = f'Looking for {self.chessboard_size[0]}x{self.chessboard_size[1]} inner corners'
            cv2.putText(display_image, hint_text, (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            return False, display_image, None
    
    def capture_image(self, corners):
        """采集图像"""
        if self.current_frame is not None:
            # 保存图像
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = os.path.join(self.save_dir, f'calib_{timestamp}.png')
            cv2.imwrite(filename, self.current_frame)
            
            # 保存到列表
            self.captured_images.append(self.current_frame.copy())
            self.captured_corners.append(corners)
            
            self.get_logger().info(f'已采集第 {len(self.captured_images)} 张图片')
            self.get_logger().info(f'保存至: {filename}')
            
            return True
        return False
    
    def calibrate_camera(self):
        """执行相机标定"""
        if len(self.captured_images) < self.min_images:
            self.get_logger().warning(
                f'采集的图片数量({len(self.captured_images)})少于最小要求({self.min_images})，建议采集更多图片以获得更好的标定效果'
            )
            response = input(f'当前采集了 {len(self.captured_images)} 张图片，是否继续标定? (y/n): ')
            if response.lower() != 'y':
                self.get_logger().info('取消标定')
                return
        
        self.get_logger().info(f'开始标定，使用 {len(self.captured_images)} 张图片...')
        
        # 准备标定数据
        obj_points = []  # 3D点
        img_points = []  # 2D点
        
        for corners in self.captured_corners:
            obj_points.append(self.objp)
            img_points.append(corners)
        
        # 获取图像尺寸
        h, w = self.captured_images[0].shape[:2]
        
        # 设置标定标志
        flags = 0
        if not self.use_rational_model:
            # 不使用有理模型（不使用k3, k4, k5, k6），只使用k1, k2, p1, p2
            flags = cv2.CALIB_FIX_K3
            self.get_logger().info('使用标准畸变模型 (k1, k2, p1, p2)')
        else:
            self.get_logger().info('使用有理畸变模型 (k1, k2, p1, p2, k3)')
        
        # 执行标定
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            obj_points, img_points, (w, h), None, None, flags=flags
        )
        
        if ret:
            self.get_logger().info('标定成功！')
            self.get_logger().info('=' * 60)
            self.get_logger().info('相机内参矩阵 (Camera Matrix):')
            self.get_logger().info(f'\n{camera_matrix}')
            self.get_logger().info('=' * 60)
            self.get_logger().info('畸变系数 (Distortion Coefficients):')
            self.get_logger().info(f'{dist_coeffs.ravel()}')
            self.get_logger().info('=' * 60)
            
            # 计算重投影误差
            total_error = 0
            for i in range(len(obj_points)):
                img_points_reprojected, _ = cv2.projectPoints(
                    obj_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
                )
                error = cv2.norm(img_points[i], img_points_reprojected, cv2.NORM_L2) / len(img_points_reprojected)
                total_error += error
            
            mean_error = total_error / len(obj_points)
            self.get_logger().info(f'平均重投影误差: {mean_error:.4f} 像素')
            self.get_logger().info('=' * 60)
            
            # 保存标定结果
            calibration_file = os.path.join(self.save_dir, 'camera_calibration.npz')
            np.savez(
                calibration_file,
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs,
                rvecs=rvecs,
                tvecs=tvecs,
                image_size=(w, h),
                mean_error=mean_error
            )
            self.get_logger().info(f'标定结果已保存至: {calibration_file}')
            
            # 保存为YAML格式(方便ROS使用)
            self.save_calibration_yaml(camera_matrix, dist_coeffs, (w, h))
            
            # 显示去畸变效果
            self.show_undistort_result(camera_matrix, dist_coeffs)
            
        else:
            self.get_logger().error('标定失败！')
    
    def save_calibration_yaml(self, camera_matrix, dist_coeffs, image_size):
        """保存标定结果为YAML格式"""
        yaml_file = os.path.join(self.save_dir, 'camera_calibration.yaml')
        
        with open(yaml_file, 'w') as f:
            f.write('%YAML:1.0\n')
            f.write('---\n')
            f.write(f'image_width: {image_size[0]}\n')
            f.write(f'image_height: {image_size[1]}\n')
            f.write('camera_matrix:\n')
            f.write('  rows: 3\n')
            f.write('  cols: 3\n')
            f.write('  data: [')
            f.write(', '.join([f'{x:.6f}' for x in camera_matrix.ravel()]))
            f.write(']\n')
            f.write('distortion_coefficients:\n')
            f.write('  rows: 1\n')
            f.write(f'  cols: {len(dist_coeffs.ravel())}\n')
            f.write('  data: [')
            f.write(', '.join([f'{x:.6f}' for x in dist_coeffs.ravel()]))
            f.write(']\n')
            f.write('distortion_model: plumb_bob\n')
        
        self.get_logger().info(f'YAML格式标定文件已保存至: {yaml_file}')
    
    def show_undistort_result(self, camera_matrix, dist_coeffs):
        """显示去畸变效果"""
        if len(self.captured_images) > 0:
            # 选择第一张图片进行演示
            img = self.captured_images[0]
            h, w = img.shape[:2]
            
            # 获取优化的相机矩阵
            new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
                camera_matrix, dist_coeffs, (w, h), 1, (w, h)
            )
            
            # 去畸变
            undistorted = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_camera_matrix)
            
            # 裁剪图像
            x, y, w, h = roi
            if w > 0 and h > 0:
                undistorted_cropped = undistorted[y:y+h, x:x+w]
            else:
                undistorted_cropped = undistorted
            
            # 显示对比
            cv2.imshow('Original Image', img)
            cv2.imshow('Undistorted Image', undistorted)
            cv2.imshow('Undistorted and Cropped', undistorted_cropped)
            
            self.get_logger().info('显示去畸变效果，按任意键或点击窗口关闭...')
            cv2.waitKey(0)
            cv2.destroyAllWindows()
    
    def mouse_callback(self, event, x, y, flags, param):
        """鼠标回调函数"""
        if event == cv2.EVENT_LBUTTONDOWN:
            # 检查是否点击了采集按钮
            if self.button_area_capture and \
               self.button_area_capture[0] <= x <= self.button_area_capture[2] and \
               self.button_area_capture[1] <= y <= self.button_area_capture[3]:
                self.capture_requested = True
                self.get_logger().info('点击了采集按钮')
            
            # 检查是否点击了退出按钮
            elif self.button_area_quit and \
                 self.button_area_quit[0] <= x <= self.button_area_quit[2] and \
                 self.button_area_quit[1] <= y <= self.button_area_quit[3]:
                self.quit_requested = True
                self.get_logger().info('点击了完成标定按钮')
    
    def draw_buttons(self, image):
        """在图像上绘制按钮"""
        h, w = image.shape[:2]
        
        # 按钮尺寸和位置
        button_width = 180
        button_height = 50
        margin = 20
        spacing = 20
        
        # 采集按钮（绿色，左侧）
        x1_capture = margin
        y1_capture = h - margin - button_height
        x2_capture = x1_capture + button_width
        y2_capture = y1_capture + button_height
        self.button_area_capture = (x1_capture, y1_capture, x2_capture, y2_capture)
        
        # 根据是否检测到棋盘格改变按钮颜色
        if self.last_ret:
            button_color_capture = (0, 200, 0)  # 亮绿色 - 可以采集
            text_color_capture = (255, 255, 255)
        else:
            button_color_capture = (100, 100, 100)  # 灰色 - 不可采集
            text_color_capture = (150, 150, 150)
        
        cv2.rectangle(image, (x1_capture, y1_capture), (x2_capture, y2_capture), 
                     button_color_capture, -1)
        cv2.rectangle(image, (x1_capture, y1_capture), (x2_capture, y2_capture), 
                     (255, 255, 255), 2)
        
        # 采集按钮文字（英文，避免乱码）
        text = f'CAPTURE ({len(self.captured_images)})'
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 2
        text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
        text_x = x1_capture + (button_width - text_size[0]) // 2
        text_y = y1_capture + (button_height + text_size[1]) // 2
        cv2.putText(image, text, (text_x, text_y), font, font_scale, 
                   text_color_capture, thickness)
        
        # 退出按钮（红色，右侧）
        x1_quit = w - margin - button_width
        y1_quit = h - margin - button_height
        x2_quit = x1_quit + button_width
        y2_quit = y1_quit + button_height
        self.button_area_quit = (x1_quit, y1_quit, x2_quit, y2_quit)
        
        cv2.rectangle(image, (x1_quit, y1_quit), (x2_quit, y2_quit), 
                     (0, 0, 200), -1)  # 红色
        cv2.rectangle(image, (x1_quit, y1_quit), (x2_quit, y2_quit), 
                     (255, 255, 255), 2)
        
        # 退出按钮文字（英文，避免乱码）
        text = 'FINISH'
        text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
        text_x = x1_quit + (button_width - text_size[0]) // 2
        text_y = y1_quit + (button_height + text_size[1]) // 2
        cv2.putText(image, text, (text_x, text_y), font, font_scale, 
                   (255, 255, 255), thickness)
        
        return image
    
    def run(self):
        """主循环"""
        window_name = 'Camera Calibration'
        window_created = False
        frame_count = 0
        display_fps = 10  # 降低显示帧率，提高性能
        frame_skip = 0
        
        self.get_logger().info('等待图像数据...')
        self.get_logger().info(f'显示帧率: {display_fps} FPS')
        
        while rclpy.ok():
            try:
                # 先处理ROS回调
                rclpy.spin_once(self, timeout_sec=0.01)
                
                if frame_count % 100 == 0 and not window_created:
                    self.get_logger().info(f'循环次数: {frame_count}, 等待图像...')
                
            except Exception as e:
                self.get_logger().error(f'spin_once错误: {str(e)}')
                break
            
            if self.current_frame is not None:
                frame_count += 1
                
                if frame_count == 1:
                    self.get_logger().info('收到第一帧图像')
                
                # 跳帧处理，降低CPU占用（仅在已有窗口时）
                # 但保持键盘响应
                skip_processing = False
                if window_created:
                    frame_skip += 1
                    if frame_skip < 3:  # 每3帧处理一次显示
                        skip_processing = True
                    else:
                        frame_skip = 0
                
                # 延迟创建窗口，确保有图像数据后再创建
                if not window_created:
                    try:
                        self.get_logger().info('正在创建OpenCV窗口...')
                        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
                        self.get_logger().info('窗口命名完成，准备调整大小...')
                        # 使用小一点的初始尺寸，避免某些显示问题
                        cv2.resizeWindow(window_name, 800, 600)
                        # 设置鼠标回调
                        cv2.setMouseCallback(window_name, self.mouse_callback)
                        self.get_logger().info('OpenCV窗口创建成功，鼠标回调已设置')
                        self.get_logger().info('💡 点击界面底部的按钮进行操作')
                        window_created = True
                    except Exception as e:
                        self.get_logger().error(f'无法创建OpenCV窗口: {str(e)}')
                        import traceback
                        traceback.print_exc()
                        self.get_logger().error('请检查DISPLAY环境变量和X11转发设置')
                        return
                # 只在不跳帧时处理图像
                if not skip_processing:
                    # 查找并绘制角点
                    ret, display_image, corners = self.find_and_draw_corners(self.current_frame)
                    
                    # 保存检测结果，供按钮使用
                    self.last_ret = ret
                    self.last_corners = corners
                    
                    # 添加信息文本（英文，避免乱码）
                    info_text = f'Captured: {len(self.captured_images)} / Min Required: {self.min_images}'
                    cv2.putText(display_image, info_text, (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    
                    if ret:
                        status_text = 'Chessboard Detected! Ready to capture'
                        cv2.putText(display_image, status_text, (10, 60), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    else:
                        status_text = 'Chessboard NOT detected'
                        cv2.putText(display_image, status_text, (10, 60), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
                    # 绘制按钮
                    display_image = self.draw_buttons(display_image)
                
                # 显示图像和处理鼠标事件（每帧都执行，不跳过）
                if window_created:
                    try:
                        # 只在处理帧时更新显示
                        if not skip_processing:
                            cv2.imshow(window_name, display_image)
                        
                        # 等待事件（包括鼠标点击）
                        cv2.waitKey(1)
                        
                        # 处理采集请求
                        if self.capture_requested:
                            self.capture_requested = False
                            if self.last_ret and self.last_corners is not None:
                                self.capture_image(self.last_corners)
                            else:
                                self.get_logger().warning('未检测到棋盘格，无法采集图片！')
                        
                        # 处理退出请求
                        if self.quit_requested:
                            self.get_logger().info('用户点击完成标定按钮，退出采集模式...')
                            break
                            
                    except Exception as e:
                        self.get_logger().error(f'OpenCV显示错误: {str(e)}')
                        import traceback
                        traceback.print_exc()
                        break
        
        # 安全关闭窗口
        if window_created:
            try:
                cv2.destroyAllWindows()
                cv2.waitKey(1)  # 确保窗口关闭
            except:
                pass
        
        # 如果采集了图片，进行标定
        if len(self.captured_images) > 0:
            self.calibrate_camera()
        else:
            self.get_logger().warning('未采集任何图片，无法进行标定！')


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = CameraCalibrationNode()
        node.run()
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('接收到键盘中断信号')
    except Exception as e:
        if node:
            node.get_logger().error(f'发生错误: {str(e)}')
        import traceback
        traceback.print_exc()
    finally:
        # 确保清理OpenCV窗口
        try:
            cv2.destroyAllWindows()
        except:
            pass
        
        # 关闭ROS2
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
