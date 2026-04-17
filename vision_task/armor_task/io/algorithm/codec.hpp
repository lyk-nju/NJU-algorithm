#pragma once

#include "../structs/protocol.hpp"

#include <string>

// 下位机通信协议的纯编解码函数（与传输层无关，可单测）。
// 旧名：io/usb/unpack.hpp。
namespace io::codec
{

PlayerMode decode_mode(int mode_value);

std::string encode(const Vision2Cboard &data);

bool decode(const std::string &line, Cboard2Vision &out);

} // namespace io::codec
