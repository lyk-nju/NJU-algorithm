#!/bin/bash

# 海康相机参数快速调试脚本
# 用于实时调整曝光时间和增益参数

echo "=========================================="
echo "   海康相机参数调试工具"
echo "=========================================="
echo ""

# 检查节点是否运行
if ! ros2 node list | grep -q "/hik_camera"; then
    echo "❌ 错误: 相机节点未运行"
    echo "请先启动相机节点:"
    echo "  ros2 launch hik_camera hik_camera.launch.py"
    exit 1
fi

echo "✅ 检测到相机节点正在运行"
echo ""

# 获取当前参数值
echo "📊 当前参数值:"
CURRENT_EXPOSURE=$(ros2 param get /hik_camera exposure_time 2>/dev/null | grep -oP '(?<=Double value is: )\d+\.?\d*')
CURRENT_GAIN=$(ros2 param get /hik_camera gain 2>/dev/null | grep -oP '(?<=Double value is: )\d+\.?\d*')

echo "  曝光时间 (exposure_time): ${CURRENT_EXPOSURE} μs"
echo "  增益 (gain): ${CURRENT_GAIN} dB"
echo ""

# 菜单选择
echo "请选择操作:"
echo "  1) 调整曝光时间"
echo "  2) 调整增益"
echo "  3) 同时调整曝光和增益"
echo "  4) 使用预设配置"
echo "  5) 查看参数范围"
echo "  6) 退出"
echo ""
read -p "请输入选项 (1-6): " choice

case $choice in
    1)
        echo ""
        echo "💡 曝光时间建议:"
        echo "  - 暗环境: 10000-30000 μs"
        echo "  - 正常光照: 5000-10000 μs"
        echo "  - 高速抓拍: 100-1000 μs"
        echo ""
        read -p "输入新的曝光时间 (μs): " exposure
        echo "设置曝光时间为: ${exposure} μs"
        ros2 param set /hik_camera exposure_time $exposure
        ;;
    
    2)
        echo ""
        echo "💡 增益建议:"
        echo "  - 良好光照: 0-10 dB (最小噪声)"
        echo "  - 中等光照: 10-16 dB"
        echo "  - 低光照: 16-24 dB (噪声增加)"
        echo ""
        read -p "输入新的增益值 (dB): " gain
        echo "设置增益为: ${gain} dB"
        ros2 param set /hik_camera gain $gain
        ;;
    
    3)
        echo ""
        read -p "输入新的曝光时间 (μs): " exposure
        read -p "输入新的增益值 (dB): " gain
        echo "设置曝光时间为: ${exposure} μs"
        echo "设置增益为: ${gain} dB"
        ros2 param set /hik_camera exposure_time $exposure
        ros2 param set /hik_camera gain $gain
        ;;
    
    4)
        echo ""
        echo "预设配置:"
        echo "  1) 暗环境 (曝光20000μs, 增益18dB)"
        echo "  2) 正常环境 (曝光8000μs, 增益12dB)"
        echo "  3) 高速抓拍 (曝光1000μs, 增益10dB)"
        echo "  4) 低噪声 (曝光5000μs, 增益5dB)"
        read -p "选择预设 (1-4): " preset
        
        case $preset in
            1)
                ros2 param set /hik_camera exposure_time 20000
                ros2 param set /hik_camera gain 18.0
                echo "✅ 已应用暗环境配置"
                ;;
            2)
                ros2 param set /hik_camera exposure_time 8000
                ros2 param set /hik_camera gain 12.0
                echo "✅ 已应用正常环境配置"
                ;;
            3)
                ros2 param set /hik_camera exposure_time 1000
                ros2 param set /hik_camera gain 10.0
                echo "✅ 已应用高速抓拍配置"
                ;;
            4)
                ros2 param set /hik_camera exposure_time 5000
                ros2 param set /hik_camera gain 5.0
                echo "✅ 已应用低噪声配置"
                ;;
            *)
                echo "❌ 无效选择"
                ;;
        esac
        ;;
    
    5)
        echo ""
        echo "📋 参数范围信息:"
        echo ""
        echo "曝光时间 (exposure_time):"
        ros2 param describe /hik_camera exposure_time
        echo ""
        echo "增益 (gain):"
        ros2 param describe /hik_camera gain
        ;;
    
    6)
        echo "退出"
        exit 0
        ;;
    
    *)
        echo "❌ 无效选择"
        exit 1
        ;;
esac

echo ""
echo "=========================================="
echo "✅ 参数调整完成"
echo ""
echo "查看更新后的参数:"
NEW_EXPOSURE=$(ros2 param get /hik_camera exposure_time 2>/dev/null | grep -oP '(?<=Double value is: )\d+\.?\d*')
NEW_GAIN=$(ros2 param get /hik_camera gain 2>/dev/null | grep -oP '(?<=Double value is: )\d+\.?\d*')
echo "  曝光时间: ${NEW_EXPOSURE} μs"
echo "  增益: ${NEW_GAIN} dB"
echo ""
echo "💡 提示: 使用 rqt 可以实时查看图像效果:"
echo "  ros2 run rqt_image_view rqt_image_view"
echo "=========================================="