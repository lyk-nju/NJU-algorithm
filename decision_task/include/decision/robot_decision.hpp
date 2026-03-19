#ifndef DECISION_ROBOT_DECISION_HPP
#define DECISION_ROBOT_DECISION_HPP

#include "decision/structs.h"

#include <cmath>
#include <cfloat>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace rdsys
{

/**
 * @brief 裁判系统信息处理类（比赛时间等）
 */
class GameHandler
{
public:
    GameHandler() = default;
    void update(int& gameTime);

private:
    std::chrono::time_point<std::chrono::system_clock, std::chrono::microseconds> lastUpdateTime;
    int gameTime = -1;
};

/**
 * @brief 决策系统类：路径点 + 决策 JSON 规则
 */
class RobotDecisionSys
{
public:
    RobotDecisionSys(float distance_thr, float seek_thr, float real_width, float real_height,
                    const std::string& map_path, float step_distance, float car_seek_fov);
    ~RobotDecisionSys();

    bool decodeWayPoints(const std::string& filePath);
    bool decodeDecisions(const std::string& filePath);

    int checkNowWayPoint(float x, float y);
    int checkNowWayPoint(const RobotPosition& pos);

    /** 决策：仅基于当前路径点、自身血量、比赛时间（及可选模式）；无前哨/基地/友敌位置 */
    std::shared_ptr<Decision> decide(int wayPointID, int robot_mode, int hp, int nowtime,
                                     std::vector<RobotPosition>& friendPositions,
                                     std::vector<RobotPosition>& enemyPositions,
                                     std::vector<int>& availableDecisionID,
                                     std::map<int, int>& id_pos_f,
                                     std::map<int, int>& id_pos_e);

    std::vector<std::shared_ptr<WayPoint>> calculatePath(int startWayPointID, int endWayPointID);
    std::shared_ptr<WayPoint> getWayPointByID(int id);
    std::shared_ptr<Decision> getDecisionByID(int id);

    float getDistanceTHR() const;
    void setDistanceTHR(float thr);
    float getSeekTHR() const;
    void setSeekTHR(float thr);

    std::vector<std::shared_ptr<WayPoint>> wayPointMap;
    std::vector<std::shared_ptr<Decision>> decisions;

private:
    int calculatePosition(RobotPosition& pos);
    cv::Point2i createEndPointByTheta(double x1, double y1, double theta, int length);
    cv::Point2i createEndPointByTheta(cv::Point start, double theta, int length);
    bool checkBlock(cv::Point start, double theta, int distance);

    std::string map_path_;
    float real_width_ = 0.0f;
    float real_height_ = 0.0f;
    float step_distance_ = 0.0f;
    float car_seek_fov_ = 0.0f;
    float _distance_THR = 0.0f;
    float _seek_THR = 5.0f;

    std::map<int, std::vector<int>> connection_map;
    std::shared_ptr<GameHandler> myGameHandler;

    cv::Mat decisionMap;
    cv::Mat decisionMap_Gray;
};

} // namespace rdsys

#endif // DECISION_ROBOT_DECISION_HPP
