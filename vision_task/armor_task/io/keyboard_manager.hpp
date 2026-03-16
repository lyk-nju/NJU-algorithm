#ifndef KEYBOARD_MANAGER_HPP
#define KEYBOARD_MANAGER_HPP

#include "serial_manager.hpp"
#include <atomic>
#include <functional>
#include <mutex>
#include <thread>

namespace io
{
/**
 * @brief 键盘管理器类，用于非阻塞键盘输入处理
 *
 * 功能：
 * - 设置终端为非阻塞模式
 * - 读取键盘输入
 * - 处理键盘命令（yaw/pitch 控制）
 * - 线程安全地获取当前命令
 */
class KeyboardManager
{
  public:
    /**
     * @brief 构造函数
     * @param step 每次按键调整的步长（度），默认 5.0
     */
    KeyboardManager(float step = 5.0f);

    /**
     * @brief 析构函数，自动恢复终端设置
     */
    ~KeyboardManager();

    /**
     * @brief 初始化键盘管理器（设置终端为非阻塞模式）
     * @return 是否成功初始化
     */
    bool init();

    /**
     * @brief 恢复终端设置
     */
    void restore();

    /**
     * @brief 启动键盘输入处理线程
     * @param running 运行标志（原子变量）
     * @param on_quit 退出回调函数（可选）
     */
    void start(std::atomic<bool> &running, std::function<void()> on_quit = nullptr);

    /**
     * @brief 停止键盘输入处理线程
     */
    void stop();

    /**
     * @brief 获取当前命令（线程安全）
     * @return 当前命令的副本
     */
    Command get_command();

    /**
     * @brief 设置命令（线程安全）
     * @param cmd 要设置的命令
     */
    void set_command(const Command &cmd);

    /**
     * @brief 重置命令（yaw 和 pitch 设为 0）
     */
    void reset_command();

    /**
     * @brief 打印键盘控制说明
     */
    static void print_controls();

    /**
     * @brief 非阻塞读取单个字符
     * @return 读取到的字符，如果没有输入则返回 0
     */
    static char read_key();

  private:
    /**
     * @brief 设置终端为非阻塞模式
     */
    void set_terminal_non_blocking();

    /**
     * @brief 键盘输入处理线程函数
     */
    void keyboard_thread_func(std::atomic<bool> &running, std::function<void()> on_quit);

    float step_;                   // 每次调整的步长
    mutable std::mutex cmd_mutex_; // 命令互斥锁
    Command cmd_;                  // 当前命令
    bool shoot_once_pending_;      // 单次射击标志
    std::thread keyboard_thread_;  // 键盘输入处理线程
    bool initialized_;             // 是否已初始化
    bool terminal_restored_;       // 终端是否已恢复
};

} // namespace io

#endif // KEYBOARD_MANAGER_HPP
