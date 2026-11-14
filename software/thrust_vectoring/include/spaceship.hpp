#pragma once

#include "thruster.hpp"
#include "vector"

struct Spaceship {
  const std::vector<Thruster> thrusters;

  Spaceship(const std::vector<Thruster> &thrusters_) : thrusters(thrusters_) {}

  std::size_t num_thrusters() const { return thrusters.size(); }
};
