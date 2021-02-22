#pragma once

static double randf()
{
  return static_cast<double>(std::rand()) / RAND_MAX;
}