#include "armor.hpp"
#include "detector.hpp"
#include "img_tools.hpp"
#include "pnp_solver.hpp"
#include "ros2_manager.hpp"
#include "tracker.hpp"

// 相机内参
cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 610, 0, 320, 0, 613, 240, 0, 0, 1);

// 畸变系数
cv::Mat distort_coeffs = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);

int main(int argc, char *argv[])
{
    // rclcpp::init(argc, argv);

    // 创建通信节点
    auto ros_node = std::make_shared<ROS2Manager>();
    std::thread spin_thread([&]() { rclcpp::spin(ros_node); });

    // 实例化功能组建
    Detector detector("NJU_RMVision\\armor_task\\models\\yolov8_armor.onnx", "NJU_RMVision\\armor_task\\models\\tiny_resnet.onnx");
    PnpSolver pnp_solver(camera_matrix, distort_coeffs);
    Tracker tracker(3, 30);

    // 帧率计算
    int frame_count = 0;
    auto last_time = std::chrono::high_resolution_clock::now();

    while (rclcpp::ok())
    {
        cv::Mat img;
        if (ros_node->get_img(img))
        {
            frame_count++;

            ArmorArray armors = detector.detect(img);         // YOLO + CV
            int success = pnp_solver.solveArmorArray(armors); // 姿态解算
            // tools::plotArmors(armors, img);                   // 绘制带框的识别结果(将装甲板的框叠加在img中)

            Armor target = tracker.evaluate(armors); // 打分筛选
            tracker.update_tracker(armors);
            Armor predict = tracker.get_current_target(); // 卡尔曼预测
            tools::plotSingleArmor(target, predict, img);

            // ros_node->publish_armor_result({predict}); // 发布结果
            // std::cout << "success detect:" << success << std::endl;
            std::cout << predict.center - target.center << std::endl;

            // 计算并输出处理帧率
            auto now = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double>(now - last_time).count();
            if (elapsed >= 1.0) // 每秒输出一次
            {
                double fps = frame_count / elapsed;
                std::cout << "Processing FPS: " << fps << ", Resolution: " << img.cols << "x" << img.rows << std::endl;
                frame_count = 0;
                last_time = now;
            }
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
