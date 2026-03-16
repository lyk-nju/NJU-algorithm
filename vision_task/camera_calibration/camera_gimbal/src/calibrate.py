import cv2
import numpy as np
import glob
import os
from scipy.spatial.transform import Rotation as R

# ================= 配置区域 =================
INPUT_FOLDER = "assets/img_with_q"
CAMERA_CONFIG_PATH = "configs/camera_params.yaml"

# --- 修改 1: 调整角点数量 ---
# 如果你有 12x7 个方格，这里填 11 和 6
# 如果你确实数出了 12x7 个内角点，这里填 12 和 7
PATTERN_COLS = 11      
PATTERN_ROWS = 6      

# --- 修改 2: 填写方格边长 ---
CENTER_DISTANCE_MM = 10.0 # 单个方格的边长 (mm)

# IMU 到云台Body的安装误差 
R_gimbal2imubody = np.array([
[ 0.0, -1.0, 0.0],
    [ 1.0,  0.0, 0.0],
    [ 0.0,  0.0, 1.0]
], dtype=np.float64)
# ===========================================

def load_camera_params(yaml_path):
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"找不到相机配置文件: {yaml_path}")
    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"无法打开相机配置文件: {yaml_path}")
    camera_matrix_node = fs.getNode("camera_matrix")
    K = camera_matrix_node.mat()
    dist_coeffs_node = fs.getNode("distortion_coefficients")
    D = dist_coeffs_node.mat()
    fs.release()
    if K is None or D is None:
        raise ValueError("配置文件中缺少参数")
    print(f"成功加载相机参数: {yaml_path}")
    return K, D

def get_3d_points(rows, cols, distance):
    points = []
    for i in range(rows):
        for j in range(cols):
            points.append([j * distance, i * distance, 0])
    return np.array(points, dtype=np.float32)

def main():
    try:
        K, D = load_camera_params(CAMERA_CONFIG_PATH)
    except Exception as e:
        print(f"错误: {e}")
        return

    R_gimbal2world_list = [] 
    t_gimbal2world_list = [] 
    R_target2cam_list = []   
    t_target2cam_list = []   
    
    objp = get_3d_points(PATTERN_ROWS, PATTERN_COLS, CENTER_DISTANCE_MM)
    
    img_files = sorted(glob.glob(os.path.join(INPUT_FOLDER, "*.jpg")), 
                       key=lambda x: int(os.path.splitext(os.path.basename(x))[0]))
    
    valid_frames = 0
    print(f"\n开始处理 {len(img_files)} 组数据 (模式: 方格棋盘)...")

    for img_path in img_files:
        base_name = os.path.splitext(os.path.basename(img_path))[0]
        txt_path = os.path.join(INPUT_FOLDER, f"{base_name}.txt")
        
        if not os.path.exists(txt_path): continue
            
        img = cv2.imread(img_path)
        if img is None: continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # 棋盘格检测需要灰度图
        
        with open(txt_path, 'r') as f:
            data = f.read().strip().replace(',', ' ').split()
            w, x, y, z = map(float, data[:4])
            
        r_obj = R.from_quat([x, y, z, w]) 
        R_imubody2imuabs = r_obj.as_matrix()
        R_g2w = R_gimbal2imubody.T @ R_imubody2imuabs @ R_gimbal2imubody
        t_g2w = np.zeros((3, 1))
        
        # --- 修改 3: 替换为 findChessboardCorners ---
        # 注意: 棋盘格不需要 symmetric grid 这种 flag，常用的 flag 如下
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        ret, corners = cv2.findChessboardCorners(gray, (PATTERN_COLS, PATTERN_ROWS), flags=flags)
        
        if ret:
            # --- 修改 4: 增加亚像素优化 (关键步骤) ---
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            success, rvec, tvec = cv2.solvePnP(objp, corners, K, D, flags=cv2.SOLVEPNP_IPPE)
            
            if success:
                R_t2c, _ = cv2.Rodrigues(rvec)
                R_gimbal2world_list.append(R_g2w)
                t_gimbal2world_list.append(t_g2w)
                R_target2cam_list.append(R_t2c)
                t_target2cam_list.append(tvec)
                
                valid_frames += 1
                print(f"[OK] Frame {base_name}: Detected")
            else:
                print(f"[FAIL] Frame {base_name}: PnP Failed")
        else:
            print(f"[FAIL] Frame {base_name}: Board not found (Check rows/cols)")

    if valid_frames < 3:
        print("有效数据不足")
        return

    print(f"\n开始手眼标定 (有效帧数: {valid_frames})...")

    try:
        R_cam2gimbal, t_cam2gimbal = cv2.calibrateHandEye(
            R_gimbal2world_list, t_gimbal2world_list, 
            R_target2cam_list, t_target2cam_list,
            method=cv2.CALIB_HAND_EYE_TSAI
        )
        
        t_cam2gimbal_m = t_cam2gimbal / 1000.0

        print("\n========== 标定结果 ==========")
        print("R_camera2gimbal:\n", R_cam2gimbal)
        print("\nt_camera2gimbal (m):\n", t_cam2gimbal_m)
        
        R_gimbal2ideal_cpp = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], dtype=np.float64)
        R_check = R_gimbal2ideal_cpp @ R_cam2gimbal
        ypr = R.from_matrix(R_check).as_euler('zyx', degrees=True)
        
        print("\n========== 安装误差 ==========")
        print(f"Yaw: {ypr[0]:.2f}°, Pitch: {ypr[1]:.2f}°, Roll: {ypr[2]:.2f}°")

    except cv2.error as e:
        print(f"标定算法出错: {e}")

if __name__ == "__main__":
    main()
