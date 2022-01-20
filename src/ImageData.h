#pragma once

#include "geom.h"
#include <vector>

struct ImageData
{
  int w;
  int h;
  size_t trace_num;
  std::vector<Color> data;
};