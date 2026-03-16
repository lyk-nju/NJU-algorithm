# 从 navigation_task1 迁移到 navigation_task2 适配指南

本文档说明当前系统（已适配 **navigation_task1**）若改为使用 **navigation_task2** 时，需要做的配置与代码适配，**不包含具体代码修改**，仅作迁移清单与设计说明。

---

## 1. 当前系统与导航的对接关系（基于 task1）

根据现有代码，系统与导航的接口依赖如下：

| 模块                    | 对接方式                           | 当前假设（task1）                                                                                                                                                          |
| ----------------------- | ---------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **decision_task**       | Nav2 Action 客户端 + 反馈/状态订阅 | Action：`navigate_through_poses`；Feedback：`navigate_through_poses/_action/feedback`；Status：`navigate_through_poses/_action/status`；目标位姿 `header.frame_id = "map"` |
| **vision_task (armor)** | 订阅导航输出的速度指令             | 订阅 **绝对话题名** `/cmd_vel`                                                                                                                                             |

- **navigation_task1**：无 namespace，上述名称即实际话题/Action 名（如 `/navigate_through_poses`、`/cmd_vel`）。
- **navigation_task2**：所有节点、话题、Action 均带 **namespace**（如 `red_standard_robot1`），实际名称会带前缀。

---

## 2. navigation_task1 与 navigation_task2 的差异概览

| 项目             | navigation_task1                                   | navigation_task2                                                                |
| ---------------- | -------------------------------------------------- | ------------------------------------------------------------------------------- |
| **命名空间**     | 无，话题/Action 在根下                             | 有 namespace（默认 `red_standard_robot1`），所有 ROS 资源带前缀                 |
| **Action 名称**  | `navigate_to_pose`、`navigate_through_poses`       | 同上，但完整名为 `/<namespace>/navigate_to_pose` 等                             |
| **cmd_vel**      | 通常为 `/cmd_vel`                                  | 带 namespace，如 `/red_standard_robot1/cmd_vel`（由 fake_vel_transform 等发布） |
| **局部规划器**   | TEB (teb_local_planner)                            | pb_omni_pid_pursuit_controller                                                  |
| **里程计话题**   | `/odom`                                            | 相对 namespace：`odometry` → `/<namespace>/odometry`                            |
| **机器人基座标** | `virt_heading_frame`                               | `gimbal_yaw_fake`                                                               |
| **行为树**       | 标准 Nav2                                          | 增加 smooth_path、assisted_teleop、truncate_path_local 等                       |
| **消息类型**     | nav2_msgs（NavigateToPose / NavigateThroughPoses） | 相同，无需改消息定义                                                            |

对外“接口类型”一致，主要差异是 **namespace 导致的话题/Action 完整名称** 以及 **启动与参数体系**。

---

## 3. 迁移适配要点（按模块）

### 3.1 decision_task

- **现状**：写死使用相对名 `navigate_through_poses` 及 `navigate_through_poses/_action/feedback`、`navigate_through_poses/_action/status`。
- **接 task2 时**：
  - **方案 A（推荐）**：decision 节点与导航在 **同一 namespace** 下启动（例如也用 `red_standard_robot1`），则继续使用相对名 `navigate_through_poses` 即可，无需改话题/Action 字符串。
  - **方案 B**：decision 不在该 namespace 下时，需改为使用 **带 namespace 的完整名**，例如：
    - Action：`/red_standard_robot1/navigate_through_poses`
    - Feedback：`/red_standard_robot1/navigate_through_poses/_action/feedback`
    - Status：`/red_standard_robot1/navigate_through_poses/_action/status`
  - **建议**：将 Action/Feedback/Status 的话题名做成 **ROS 参数**（或可从 launch 传入），这样同一份代码可通过参数切换 task1（无 namespace）与 task2（带 namespace）。
- **frame_id**：当前使用 `"map"`，与 task2 的 map frame 一致，无需修改。
- **消息类型**：仍为 `nav2_msgs/action/NavigateThroughPoses`，无需改。

### 3.2 vision_task（armor / auto_aimer）

- **现状**：订阅 **绝对话题** `/cmd_vel`。
- **接 task2 时**：导航侧实际发布的是带 namespace 的 cmd_vel，例如 `/red_standard_robot1/cmd_vel`。
  - **方案 A**：将订阅话题改为可配置（如 ROS 参数），在接 task2 时设为 `/<namespace>/cmd_vel`（如 `/red_standard_robot1/cmd_vel`）。
  - **方案 B**：若 vision 节点与导航在同一 namespace 下启动，可改为订阅相对名 `cmd_vel`，由运行时解析为 `/<namespace>/cmd_vel`。
- 若保持写死 `/cmd_vel`，则只有在 task2 将 namespace 设为空或通过 remap 把 `/<namespace>/cmd_vel` 映射到 `/cmd_vel` 时才能接到导航输出。

### 3.3 启动与集成方式

- **task1**：一般为各自 launch 分别启动导航栈、decision、vision 等，无统一 namespace。
- **task2**：
  - 使用 `pb2025_nav_bringup` 的 launch（如 `rm_navigation_simulation_launch.py`、`rm_navigation_reality_launch.py`），默认传入 `namespace:=red_standard_robot1`。
  - 若希望 decision、vision 与导航使用同一 namespace，需要在 **同一 namespace** 下启动这些节点（例如在 launch 里对 decision/vision 的 `Node` 使用 `PushRosNamespace(namespace)`，且与导航使用相同 `namespace` 参数）。
- 若使用 **多机/多 namespace**，每个机器人的 decision、vision 需与对应机器人的导航 namespace 一致（或显式配置该机器人对应的 Action 与 cmd_vel 话题名）。

### 3.4 其他可能受影响的点

- **RViz / 调试**：若用 RViz 的 “Nav2 Goal” 等发目标，需注意当前 fixed frame 与 goal 的 frame_id（`map`）一致；topic 若带 namespace，在 RViz 中要选带 namespace 的 action 或 topic。
- **TF**：task2 使用 `gimbal_yaw_fake` 等 frame，若其他模块依赖 base_link 或特定 frame 名，需确认与 task2 的 TF 树一致（或做静态/动态 TF 映射）。
- **地图与定位**：task2 使用 point_lio + small_gicp 等，地图与定位数据来源与 task1 不同，但对外仍提供 `map` 与 `odom` 等标准 frame，上层只要使用 `map` 下的目标位姿即可。

---

## 4. 迁移步骤建议（仅规划，不涉及具体改代码）

1. **确定 namespace 策略**  
   - 单机：是否采用 task2 默认 `red_standard_robot1`；若采用，decision/vision 是否与导航同 namespace。

2. **decision_task**  
   - 将 `navigate_through_poses` 及其 feedback/status 话题名参数化（或通过 launch 传入）；  
   - 接 task2 时：同 namespace 则保持相对名，否则改为带 namespace 的完整名。

3. **vision_task**  
   - 将 `/cmd_vel` 改为可配置（如参数 `cmd_vel_topic`）；  
   - 接 task2 时设为 `/<namespace>/cmd_vel` 或通过同 namespace 使用相对名 `cmd_vel`。

4. **Launch 与部署**  
   - 为 task2 单独写（或复制并改）一套 launch，对 decision、vision 使用与导航相同的 namespace；  
   - 或通过参数文件为 task1/task2 提供不同的 topic/action 名称。

5. **验证**  
   - 先只启动 task2 导航栈 + 一个测试节点，确认 `navigate_through_poses`、`cmd_vel` 的完整名与 namespace；  
   - 再接入 decision、vision，确认发 goal、收 feedback、收 cmd_vel 均正常。

---

## 5. 小结

| 适配项               | 说明                                                                                             |
| -------------------- | ------------------------------------------------------------------------------------------------ |
| **消息/Action 类型** | 不变，仍为 nav2_msgs 的 NavigateToPose/NavigateThroughPoses。                                    |
| **核心改动**         | 所有与导航相关的话题/Action 名称需支持 **namespace 前缀**（或通过同 namespace 启动避免写死）。   |
| **decision_task**    | Action 与 feedback/status 话题名参数化；接 task2 时使用带 namespace 的名称或与导航同 namespace。 |
| **vision_task**      | cmd_vel 订阅话题可配置；接 task2 时使用 `/<namespace>/cmd_vel` 或同 namespace 下的 `cmd_vel`。   |
| **启动方式**         | 使用 task2 的 bringup launch 并统一 namespace；decision/vision 与导航同 namespace 可减少配置。   |

按上述清单在配置与 launch 上做好“名称与 namespace”的适配后，即可在不改 Nav2 消息定义的前提下，从 navigation_task1 切换到 navigation_task2。
