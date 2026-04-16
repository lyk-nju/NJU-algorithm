#pragma once

#include "../dataframe/struct.hpp"
#include <string>

namespace io
{
namespace unpack
{
PlayerMode decode_mode(int mode_value);
std::string encode(const Vision2Cboard &data);
bool decode(const std::string &line, Cboard2Vision &out);
} // namespace unpack
} // namespace io

