#include "keyboard_manager.hpp"
#include <chrono>
#include <fcntl.h>
#include <iostream>
#include <opencv2/core.hpp>
#include <termios.h>
#include <unistd.h>

namespace io
{

KeyboardManager::KeyboardManager(float step) :
    step_(step * CV_PI / 180.0f), // 将输入的度数转换为弧度
    initialized_(false), terminal_restored_(false)
{
    cmd_.yaw = 0.0f;
    cmd_.pitch = 0.0f;
    cmd_.valid = false;
    cmd_.shoot = false;
    shoot_once_pending_ = false;
}

KeyboardManager::~KeyboardManager()
{
    stop();
    if (initialized_ && !terminal_restored_)
    {
        restore();
    }
}

bool KeyboardManager::init()
{
    if (initialized_)
    {
        return true;
    }

    set_terminal_non_blocking();
    initialized_ = true;
    terminal_restored_ = false;
    return true;
}

void KeyboardManager::restore()
{
    if (!initialized_ || terminal_restored_)
    {
        return;
    }

    struct termios ttystate;
    tcgetattr(STDIN_FILENO, &ttystate);
    ttystate.c_lflag |= (ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
    fcntl(STDIN_FILENO, F_SETFL, 0);
    terminal_restored_ = true;
}

void KeyboardManager::start(std::atomic<bool> &running, std::function<void()> on_quit)
{
    if (!initialized_)
    {
        init();
    }

    if (keyboard_thread_.joinable())
    {
        return; // 线程已在运行
    }

    keyboard_thread_ = std::thread(&KeyboardManager::keyboard_thread_func, this, std::ref(running), on_quit);
}

void KeyboardManager::stop()
{
    if (keyboard_thread_.joinable())
    {
        keyboard_thread_.join();
    }
}

Command KeyboardManager::get_command()
{
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    Command cmd = cmd_;
    if (shoot_once_pending_)
    {
        cmd_.shoot = false;
        shoot_once_pending_ = false;
    }
    return cmd;
}

void KeyboardManager::set_command(const Command &cmd)
{
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd_ = cmd;
}

void KeyboardManager::reset_command()
{
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd_.yaw = 0.0f;
    cmd_.pitch = 0.0f;
    cmd_.valid = false;
    cmd_.shoot = false;
}

void KeyboardManager::print_controls()
{
    std::cout << "\n=== Interactive Control Mode ===" << std::endl;
    std::cout << "Keyboard Controls:" << std::endl;
    std::cout << "  'a' - Yaw +5 degrees" << std::endl;
    std::cout << "  'd' - Yaw -5 degrees" << std::endl;
    std::cout << "  'w' - Pitch +5 degrees (up)" << std::endl;
    std::cout << "  's' - Pitch -5 degrees (down)" << std::endl;
    std::cout << "  'p' - Shoot once" << std::endl;
    std::cout << "  'q' - Quit" << std::endl;
}

char KeyboardManager::read_key()
{
    char c;
    if (read(STDIN_FILENO, &c, 1) == 1)
    {
        return c;
    }
    return 0;
}

void KeyboardManager::set_terminal_non_blocking()
{
    struct termios ttystate;
    tcgetattr(STDIN_FILENO, &ttystate);
    ttystate.c_lflag &= ~(ICANON | ECHO);
    ttystate.c_cc[VMIN] = 0;
    ttystate.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
}

void KeyboardManager::keyboard_thread_func(std::atomic<bool> &running, std::function<void()> on_quit)
{
    while (running)
    {
        char key = read_key();
        if (key != 0)
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);

            switch (key)
            {
            case 'a':
            case 'A':
                cmd_.yaw += step_;
                cmd_.valid = true;
                std::cout << "[Keyboard] Yaw increased: " << (cmd_.yaw * 180.0f / CV_PI) << " degrees (" << cmd_.yaw << " rad)" << std::endl;
                break;
            case 'd':
            case 'D':
                cmd_.yaw -= step_;
                cmd_.valid = true;
                std::cout << "[Keyboard] Yaw decreased: " << (cmd_.yaw * 180.0f / CV_PI) << " degrees (" << cmd_.yaw << " rad)" << std::endl;
                break;
            case 'w':
            case 'W':
                cmd_.pitch += step_;
                cmd_.valid = true;
                std::cout << "[Keyboard] Pitch increased: " << (cmd_.pitch * 180.0f / CV_PI) << " degrees (" << cmd_.pitch << " rad)" << std::endl;
                break;
            case 's':
            case 'S':
                cmd_.pitch -= step_;
                cmd_.valid = true;
                std::cout << "[Keyboard] Pitch decreased: " << (cmd_.pitch * 180.0f / CV_PI) << " degrees (" << cmd_.pitch << " rad)" << std::endl;
                break;
            case 'p':
            case 'P':
                cmd_.shoot = true;
                cmd_.valid = true;
                shoot_once_pending_ = true;
                std::cout << "[Keyboard] Shoot once" << std::endl;
                break;
            case 'q':
            case 'Q':
                std::cout << "\n[Keyboard] Quitting..." << std::endl;
                running = false;
                if (on_quit)
                {
                    on_quit();
                }
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

} // namespace io
