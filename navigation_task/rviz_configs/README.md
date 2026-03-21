# RViz Configurations for Remote Visualization

This folder contains the RViz2 configuration files extracted from the `pb_rm_simulation` project. These files allow you (or an AI agent) to quickly set up data visualization on a remote computer without needing to compile the entire workspace.

## 📁 Files included

- `fastlio.rviz`: Specifically designed for visualizing SLAM and Odometry when using the **Fast-LIO** algorithm. Includes point cloud and trajectory displays.
- `pointlio.rviz`: Similar to the above, but tuned for the **Point-LIO** algorithm.
- `nav2.rviz`: Designed for visualizing **Navigation2** data, including global/local costmaps, global plans, current poses, and setting 2D Navigation Goals.
- `sim_rviz2.rviz`: General visualization config mainly used in Gazebo simulations.

## 📡 Remote Visualization Support (ROS 2)

**Yes, remote visualization is natively supported!** 

Since the project uses ROS 2, remote communication is handled automatically by the DDS middleware. You do not need any special packages or plugins. 

### Prerequisites for Remote PC
1. Both the robot (Nvidia PC) and the remote computer MUST be on the **same local network (LAN)**.
2. Both computers MUST have the same `ROS_DOMAIN_ID` (default is usually 0). 
   *(You can check this by running `echo $ROS_DOMAIN_ID` in the terminal).*
3. The remote computer must have ROS 2 and `rviz2` installed.

## 🚀 How to Run (Instructions for Agent/User)

To start visualizing the data from the robot, simply run `rviz2` and point it to the relevant config file using the `-d` flag.

**For LIO / SLAM Visualization:**
```bash
ros2 run rviz2 rviz2 -d fastlio.rviz
# or
ros2 run rviz2 rviz2 -d pointlio.rviz
```

**For Navigation2 Visualization:**
```bash
ros2 run rviz2 rviz2 -d nav2.rviz
```

*(Note: Ensure you are in the directory containing these files when running the commands, or provide the absolute path).*
