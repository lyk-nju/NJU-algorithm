#ifndef DECISION_JUDGE_PARSER_HPP
#define DECISION_JUDGE_PARSER_HPP

#include "decision/structs.h"

#include <vector>

namespace io
{
struct JudgerData
{
    int game_time = 0;           // 比赛时间
    int self_hp = 0;             // 自身血量
};
} // namespace io

namespace decision
{

struct ParsedJudgeInfo
{
    int game_time = 0;
    int self_hp = 0;
    // 由节点用 Nav2 feedback 填充
    float self_x = 0.0f; 
    float self_y = 0.0f;
    std::vector<rdsys::RobotPosition> friend_positions;
    std::vector<rdsys::RobotPosition> enemy_positions;
};

/** 从 JudgerData 解析出 ParsedJudgeInfo；当前协议仅含 game_time、self_hp */
inline bool parse_judger_data(const io::JudgerData& raw, ParsedJudgeInfo& out)
{
    out.game_time = raw.game_time;
    out.self_hp = raw.self_hp;
    out.friend_positions.clear();
    out.enemy_positions.clear();
    return true;
}

} // namespace decision

#endif // DECISION_JUDGE_PARSER_HPP
