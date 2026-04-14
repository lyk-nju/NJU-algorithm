#include "io/usb/cboard.hpp"
#include "io/ros2_manager.hpp"
#include "tasks/aimer.hpp"
#include "tasks/detector.hpp"
#include "tasks/pnp_solver.hpp"
#include "tasks/shooter.hpp"
#include "tasks/tracker.hpp"
#include "tools/draw.hpp"
#include "tools/pharser.hpp"
#include "tools/transfer.hpp"
#include "tools/visualizer.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <thread>

using namespace armor_task;

//鍙槸妗嗘灦
int main(){
    std::string test_config_path = "../config/deploy_test.yaml";
    TestConfig test_config = load_deploy_test_config(test_config_path);
    std::string yolo_model_path = test_config.yolo_model_path;
    std::string config_path = test_config.config_path;
    std::string send_port = test_config.send_port;
    std::string receive_port = test_config.receive_port;

    Cboard cboard(config_path);
    Detector detector(yolo_model_path);
    PnpSolver pnp_solver(config_path);

   
    Tracker tracker(config_path, pnp_solver); 
    Aimer aimer(config_path);
    Shooter shooter(config_path);
    

    cv::Mat camera_matrix = camera_params.first;
    cv::Mat distort_coeffs = camera_params.second;

    camera.read(img,t);
    auto q = cboard.q(t); //璇诲彇绾跨▼鐨勫紑鍚皝瑁呭埌cboard鐨勬瀯閫犲嚱鏁伴噷闈?

    solver.set_R_gimbal2world(q);
    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    auto armors = yolo.detect(img);

    bool enemy_is_red = cboard.set_enemy();
    auto targets = tracker.track(armors, t, enemy_is_red); 
    
    const io::JudgerData jd= cboard.judger;
    double bullet_speed = jd.bullet_speed;
    io::Vision2Cboard cmd2cboard = aimer.aim(targets, t, bullet_speed);
    cmd2cboard.shoot = shooter.shoot(cmd2cboard,aimer,targets,ypr);

    cboard.send(cmd2cboard);
    
}

