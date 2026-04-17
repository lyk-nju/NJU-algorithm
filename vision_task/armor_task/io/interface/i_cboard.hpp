#pragma once

#include "../structs/protocol.hpp"

#include <Eigen/Geometry>
#include <chrono>

namespace io
{

/**
 * 下位机通信抽象。把"串口 / 虚拟串口 / 仿真"对上层做屏蔽。
 *
 * 关键能力：
 *   - mode() / judger()                   ：当前上下位机交互状态
 *   - gimbal_quat_at(t)                   ：查询 t 时刻的云台姿态四元数（slerp 插值）
 *   - send(...)                           ：下发视觉决策数据帧
 *   - is_connected() / has_quat_data()    ：显式暴露连通性 / IMU 数据可用性，
 *                                           避免上层靠 "gimbal_quat_at 返回 Identity"
 *                                           这种隐式信号做判断
 */
class ICboard
{
  public:
    virtual ~ICboard() = default;

    virtual PlayerMode mode() const = 0;
    virtual JudgerData judger() const = 0;

    /**
     * 查询 t 时刻的云台四元数（slerp 插值），非破坏性。
     *
     * 若 has_quat_data() == false（尚未收到任何 IMU 采样），实现应返回
     * Eigen::Quaterniond::Identity()；越界时实现应 clamp 到缓冲端点。
     * 上层若关心"数据是否可信"，应先检查 has_quat_data()。
     */
    virtual Eigen::Quaterniond gimbal_quat_at(std::chrono::steady_clock::time_point t) const = 0;

    virtual void send(const Vision2Cboard &vision2cboard) = 0;

    /// 下位机通信链路当前是否可用（串口是否成功打开）
    virtual bool is_connected() const = 0;

    /// 是否已经至少收到过一条 IMU 四元数采样（用于判断 gimbal_quat_at 的返回值是否可信）
    virtual bool has_quat_data() const = 0;
};

} // namespace io
