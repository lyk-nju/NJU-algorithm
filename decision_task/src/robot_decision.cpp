#include "decision/robot_decision.hpp"

#include <fstream>
#include <algorithm>

#include <nlohmann/json.hpp>

namespace rdsys
{

void GameHandler::update(int& gameTime)
{
    lastUpdateTime = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now());
    this->gameTime = gameTime;
}

int RobotDecisionSys::calculatePosition(RobotPosition& pos)
{
    double distance = static_cast<double>(FLT_MAX);
    int id = -1;
    for (const auto& it : wayPointMap)
    {
        double tempDistance = std::sqrt(std::pow(static_cast<double>(it->x - pos.x), 2) +
                                        std::pow(static_cast<double>(it->y - pos.y), 2));
        if (tempDistance < distance)
        {
            distance = tempDistance;
            id = it->id;
        }
    }
    return (distance > static_cast<double>(_distance_THR)) ? -1 : id;
}

std::vector<std::shared_ptr<WayPoint>> RobotDecisionSys::calculatePath(int startWayPointID, int endWayPointID)
{
    std::vector<std::shared_ptr<WayPoint>> result;
    std::vector<int> container = { startWayPointID };
    std::map<int, bool> waypoint_flag;
    waypoint_flag[startWayPointID] = true;
    bool flag = true;
    while (flag)
    {
        int current_waypoint = container.back();
        auto iter = connection_map.find(current_waypoint);
        if (iter == connection_map.end())
            break;
        bool flag2 = false;
        for (int next_id : iter->second)
        {
            if (waypoint_flag.find(next_id) == waypoint_flag.end())
            {
                waypoint_flag[next_id] = true;
                container.push_back(next_id);
                flag2 = true;
                flag = (next_id != endWayPointID);
                break;
            }
        }
        if (!flag2)
            container.pop_back();
        if (waypoint_flag.size() >= wayPointMap.size())
            break;
    }
    for (size_t i = 1; i < container.size(); ++i)
        result.push_back(getWayPointByID(container[i]));
    return result;
}

cv::Point2i RobotDecisionSys::createEndPointByTheta(double x1, double y1, double theta, int length)
{
    if (theta < 0.0)
        theta = 2.0 * CV_PI - std::abs(theta);
    return cv::Point2i(static_cast<int>(std::round(x1 + length * std::cos(theta))),
                       static_cast<int>(std::round(y1 + length * std::sin(theta))));
}

cv::Point2i RobotDecisionSys::createEndPointByTheta(cv::Point start, double theta, int length)
{
    if (theta < 0.0)
        theta = 2.0 * CV_PI - std::abs(theta);
    return cv::Point2i(static_cast<int>(std::round(start.x + length * std::cos(theta))),
                       static_cast<int>(std::round(start.y + length * std::sin(theta))));
}

bool RobotDecisionSys::checkBlock(cv::Point start, double theta, int distance)
{
    if (decisionMap_Gray.empty())
        return false;
    int temp_step = static_cast<int>(step_distance_ * (1080.0 / real_height_));
    for (double distance_step = temp_step; distance_step < distance; distance_step += temp_step)
    {
        cv::Point2i check_point = createEndPointByTheta(start, theta, static_cast<int>(distance_step));
        if (check_point.y >= 0 && check_point.y < decisionMap_Gray.rows &&
            check_point.x >= 0 && check_point.x < decisionMap_Gray.cols)
        {
            if (static_cast<int>(decisionMap_Gray.at<uchar>(check_point.y, check_point.x)) < 10)
                return true;
        }
    }
    return false;
}

RobotDecisionSys::RobotDecisionSys(float distance_thr, float seek_thr, float real_width, float real_height,
                                 const std::string& map_path, float step_distance, float car_seek_fov)
    : map_path_(map_path)
    , real_width_(real_width)
    , real_height_(real_height)
    , step_distance_(step_distance)
    , car_seek_fov_(car_seek_fov)
    , _distance_THR(distance_thr)
    , _seek_THR(seek_thr)
{
    myGameHandler = std::make_shared<GameHandler>();
    decisionMap = cv::imread(map_path_);
    if (!decisionMap.empty())
    {
        cv::resize(decisionMap, decisionMap,
                   cv::Size(static_cast<int>(real_width_ / real_height_ * 1080.0), 1080));
        cv::cvtColor(decisionMap, decisionMap_Gray, cv::COLOR_BGR2GRAY);
    }
}

RobotDecisionSys::~RobotDecisionSys() = default;

bool RobotDecisionSys::decodeWayPoints(const std::string& filePath)
{
    std::ifstream jsonFile(filePath);
    if (!jsonFile.is_open())
    {
        return false;
    }
    nlohmann::json j;
    try
    {
        jsonFile >> j;
    }
    catch (const std::exception&)
    {
        jsonFile.close();
        return false;
    }
    jsonFile.close();

    if (!j.contains("data") || !j["data"].is_array())
        return false;

    for (const auto& item : j["data"])
    {
        auto wayPoint = std::make_shared<WayPoint>();
        wayPoint->id = item.value("id", 0);
        wayPoint->type = item.value("type", 0);
        wayPoint->x = item.value("x", 0.0f);
        wayPoint->y = item.value("y", 0.0f);
        wayPoint->theta = item.value("angle", 0.0);
        if (item.contains("connect") && item["connect"].is_array())
        {
            for (const auto& c : item["connect"])
                wayPoint->connection.push_back(c.get<int>());
        }
        if (item.contains("enemyWeights") && item["enemyWeights"].is_array())
        {
            int idx = 0;
            for (const auto& w : item["enemyWeights"])
                wayPoint->enemyWeights[idx++] = w.get<int>();
        }
        wayPointMap.push_back(wayPoint);
        connection_map[wayPoint->id] = wayPoint->connection;
    }
    return true;
}

bool RobotDecisionSys::decodeDecisions(const std::string& filePath)
{
    std::ifstream jsonFile(filePath);
    if (!jsonFile.is_open())
    {
        return false;
    }
    nlohmann::json j;
    try
    {
        jsonFile >> j;
    }
    catch (const std::exception&)
    {
        jsonFile.close();
        return false;
    }
    jsonFile.close();

    if (!j.contains("data") || !j["data"].is_array())
        return false;

    for (const auto& item : j["data"])
    {
        auto decision = std::make_shared<Decision>();
        decision->id = item.value("id", 0);
        decision->name = item.value("name", std::string(""));
        if (item.contains("wayPointID") && item["wayPointID"].is_array())
        {
            for (const auto& w : item["wayPointID"])
                decision->wayPointID.push_back(w.get<int>());
        }
        decision->weight = item.value("weight", 0);
        decision->start_time = item.value("start_time", -1);
        decision->end_time = item.value("end_time", -1);
        decision->robot_mode = item.value("robot_mode", -1);
        decision->_minHP = item.value("minHP", -1);
        decision->_maxHP = item.value("maxHP", -1);
        decision->decide_wayPoint = item.value("decide_wayPoint", -1);
        decision->out_post_HP_min = item.value("out_post_HP_min", -1);
        decision->base_HP_min = item.value("base_HP_min", -1);
        decision->if_succession = item.value("if_succession", false);
        decision->if_reverse = item.value("if_reverse", false);

        if (item.contains("enemyPosition") && item["enemyPosition"].is_array())
        {
            for (const auto& row : item["enemyPosition"])
            {
                std::vector<int> temp;
                if (row.is_array())
                    for (const auto& v : row)
                        temp.push_back(v.get<int>());
                decision->enemy_position.push_back(std::move(temp));
            }
        }
        if (item.contains("friendPosition") && item["friendPosition"].is_array())
        {
            for (const auto& row : item["friendPosition"])
            {
                std::vector<int> temp;
                if (row.is_array())
                    for (const auto& v : row)
                        temp.push_back(v.get<int>());
                decision->friend_position.push_back(std::move(temp));
            }
        }
        decisions.push_back(decision);
    }
    return true;
}

int RobotDecisionSys::checkNowWayPoint(float x, float y)
{
    RobotPosition pos;
    pos.x = x;
    pos.y = y;
    return calculatePosition(pos);
}

int RobotDecisionSys::checkNowWayPoint(const RobotPosition& pos)
{
    RobotPosition p = pos;
    return calculatePosition(p);
}

std::shared_ptr<Decision> RobotDecisionSys::decide(int wayPointID, int robot_mode, int hp, int nowtime,
                                                   std::vector<RobotPosition>& friendPositions,
                                                   std::vector<RobotPosition>& enemyPositions,
                                                   std::vector<int>& availableDecisionID,
                                                   std::map<int, int>& id_pos_f,
                                                   std::map<int, int>& id_pos_e)
{
    myGameHandler->update(nowtime);
    id_pos_f.clear();
    id_pos_e.clear();
    const bool use_position_constraint = !friendPositions.empty() || !enemyPositions.empty();
    if (use_position_constraint)
    {
        for (auto& it : friendPositions)
            id_pos_f[it.robot_id] = calculatePosition(it);
        for (auto& it : enemyPositions)
            id_pos_e[it.robot_id] = calculatePosition(it);
    }

    std::vector<std::shared_ptr<Decision>> tempDecision;
    for (const auto& it : decisions)
    {
        // 1. 当前路径点是否在决策要求的 wayPointID 内
        if (!it->wayPointID.empty() && it->wayPointID[0] != -1)
        {
            bool check = false;
            for (int jt : it->wayPointID)
            {
                if (jt == wayPointID)
                {
                    check = true;
                    break;
                }
            }
            if (!check)
                continue;
        }
        // 2. 自身血量、比赛时间、模式（-1 表示不限制）
        if ((it->robot_mode != -1 && it->robot_mode != robot_mode) ||
            (it->_maxHP != -1 && hp > it->_maxHP) ||
            (it->_minHP != -1 && hp <= it->_minHP) ||
            (it->end_time != -1 && nowtime > it->end_time) ||
            (it->start_time != -1 && nowtime <= it->start_time))
        {
            continue;
        }
        // 3. 友方/敌方位置约束：当前仅用血量与时间，无位置数据时直接通过
        if (!use_position_constraint)
        {
            tempDecision.push_back(it);
            continue;
        }
        bool fpFLAG = true;
        for (size_t i = 0; i < it->friend_position.size(); ++i)
        {
            auto fit = id_pos_f.find(static_cast<int>(i));
            if (fit == id_pos_f.end())
                continue;
            int temp_pos = fit->second;
            if (it->friend_position[i].empty() || it->friend_position[i][0] == -1)
                continue;
            if (std::find(it->friend_position[i].begin(), it->friend_position[i].end(), temp_pos) ==
                it->friend_position[i].end())
            {
                fpFLAG = false;
                break;
            }
        }
        bool epFLAG = true;
        for (size_t i = 0; i < it->enemy_position.size(); ++i)
        {
            auto eit = id_pos_e.find(static_cast<int>(i));
            if (eit == id_pos_e.end())
                continue;
            int temp_pos = eit->second;
            if (it->enemy_position[i].empty() || it->enemy_position[i][0] == -1)
                continue;
            if (std::find(it->enemy_position[i].begin(), it->enemy_position[i].end(), temp_pos) ==
                it->enemy_position[i].end())
            {
                epFLAG = false;
                break;
            }
        }
        if (epFLAG && fpFLAG)
            tempDecision.push_back(it);
    }

    int max_weight = 0;
    std::shared_ptr<Decision> decision = nullptr;
    availableDecisionID.clear();
    for (const auto& it : tempDecision)
    {
        availableDecisionID.push_back(it->id);
        if (it->weight > max_weight)
        {
            max_weight = it->weight;
            decision = it;
            decision->if_auto = false;
        }
    }
    if (decision != nullptr && decision->decide_wayPoint == -1)
        decision->decide_wayPoint = wayPointID;
    return decision;
}

std::shared_ptr<WayPoint> RobotDecisionSys::getWayPointByID(int id)
{
    for (auto& it : wayPointMap)
        if (it->id == id)
            return it;
    return nullptr;
}

std::shared_ptr<Decision> RobotDecisionSys::getDecisionByID(int id)
{
    for (auto& it : decisions)
        if (it->id == id)
            return it;
    return nullptr;
}

float RobotDecisionSys::getDistanceTHR() const
{
    return _distance_THR;
}

void RobotDecisionSys::setDistanceTHR(float thr)
{
    _distance_THR = thr;
}

float RobotDecisionSys::getSeekTHR() const
{
    return _seek_THR;
}

void RobotDecisionSys::setSeekTHR(float thr)
{
    _seek_THR = thr;
}

} // namespace rdsys
