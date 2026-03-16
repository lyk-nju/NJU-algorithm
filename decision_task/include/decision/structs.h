#ifndef DECISION_STRUCTS_H
#define DECISION_STRUCTS_H

#include <map>
#include <string>
#include <vector>

namespace rdsys
{

/**
 * @brief 机器人位置信息
 */
struct RobotPosition
{
    int robot_id = -1;
    float x = 0.0f;
    float y = 0.0f;
    RobotPosition() = default;
    RobotPosition(int id, float x, float y) : robot_id(id), x(x), y(y) {}
};

/**
 * @brief 路径点信息
 */
struct WayPoint
{
    int id = 0;
    int type = 0;
    float x = 0.0f;
    float y = 0.0f;
    double theta = 0.0;
    std::map<int, int> enemyWeights;
    std::vector<int> connection;
};

/**
 * @brief 决策信息
 */
struct Decision
{
    bool if_auto = true;
    int id = 0;
    std::string name;
    std::vector<int> wayPointID;
    int weight = 0;
    int robot_mode = -1;
    int start_time = -1;
    int end_time = -1;
    int _minHP = -1;
    int _maxHP = -1;
    int out_post_HP_min = -1;
    int base_HP_min = -1;
    std::vector<std::vector<int>> enemy_position;
    std::vector<std::vector<int>> friend_position;
    int decide_mode = -1;
    int decide_wayPoint = -1;
    bool if_succession = false;
    bool if_reverse = true;
};

} // namespace rdsys

#endif // DECISION_STRUCTS_H
