fake_publish:

为了实现按键控制，我们需要引入处理终端输入（Non-blocking I/O）的逻辑。在 Linux 环境下，通常使用 termios.h 来修改终端模式，使得程序可以在不输入回车的情况下捕捉到按键。

我将代码修改为：启动一个独立的线程专门监听键盘输入，通过一个原子布尔值控制发送逻辑。
修改后的代码：fake_publish.cpp
主要改动说明：

    引入 std::thread 和 termios：

        因为 rclcpp::spin 会阻塞主线程，我们必须创建一个辅助线程 keyboard_loop 来持续监听键盘。

        termios 用于关闭终端的“行缓冲”和“回显”，这样你按一下键盘就能立即响应，不需要按 Enter。

    逻辑替换：

        删除了 elapsed % 2 == 0 的时间取模逻辑。

        新增 std::atomic<bool> keep_moving_ 标志位。

        按下 'a'：keep_moving_ 设为 true，定时器下次发布时会填入设置的数值。

        按下 'q'：keep_moving_ 设为 false，定时器下次发布 geometry_msgs::msg::Twist 的默认构造函数生成的消息（即全 0）。

    日志优化：

        为了防止终端被高频发布信息刷屏，我将普通状态日志改成了每 20 次发布打印一次，但在按键瞬间会立刻打印 RCLCPP_WARN 反馈。

运行方法：

编译后直接运行。此时终端会停留在等待状态：

    按下键盘 a：你会看到状态切换为 ACTIVE，/cmd_vel 开始发送非零数据。

    按下键盘 q：状态切换为 STOPPED，/cmd_vel 发送全 0。

您需要我帮您补充对应的 CMakeLists.txt 配置吗？