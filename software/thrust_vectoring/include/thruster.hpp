#pragma once

#include "math/vector.hpp"

struct Thruster {
  const Vector3 r;
  const Vector3 d;
  const double u_max;

  constexpr Thruster(Vector3 r_, Vector3 d_, double u_max_)
      : r(r_), d(d_.normalized()), u_max(u_max_) {
    if (u_max_ <= 0) {
      throw std::invalid_argument("Thruster u_max must be positive");
    }
  }

  const Vector3 calculate_b() const { return r.cross(d); }
};
