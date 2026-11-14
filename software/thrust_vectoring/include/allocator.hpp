#pragma once

#include "math/matrix.hpp"
#include "math/vector.hpp"
#include "spaceship.hpp"
#include <cstddef>
#include <vector>

class Allocator {
public:
  explicit Allocator(const Spaceship &ship) : ship_(ship) {
    Matrix3 M = Matrix3::zeros();
    b_.reserve(ship_.num_thrusters());
    for (const Thruster &t : ship_.thrusters) {
      Vector3 b = t.calculate_b();
      b_.push_back(b);
      M = M + Matrix3::outer(b, b);
    }
    M_inv_ = M.inverse();
  }

  std::vector<double> allocate(const Vector3 &tau_c) const {
    const Vector3 lambda = M_inv_ * tau_c;
    std::vector<double> u;
    u.reserve(b_.size());
    for (const Vector3 &b : b_) {
      u.push_back(b.dot(lambda));
    }
    return u;
  }

  Vector3 torque_from(const std::vector<double> &u) const {
    Vector3 tau{0.0, 0.0, 0.0};
    for (std::size_t i = 0; i < b_.size(); ++i) {
      tau = tau + b_[i] * u[i];
    }
    return tau;
  }

private:
  const Spaceship &ship_;
  std::vector<Vector3> b_;
  Matrix3 M_inv_;
};
