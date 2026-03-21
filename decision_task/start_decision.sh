export AMENT_PREFIX_PATH=/home/nvidia/NJU-algorithm/decision_task/install/decision:$AMENT_PREFIX_PATH
source ./install/setup.sh
ros2 launch decision pose_init_and_decision.launch.py