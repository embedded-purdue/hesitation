#pragma once

#include <cmath>
#include <iostream>
#include <stdexcept>

struct Vector3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};

  constexpr Vector3() = default;

  constexpr Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

  double dot(const Vector3 &o) const { return (x * o.x + y * o.y + z * o.z); }

  constexpr double norm_squared() const { return x * x + y * y + z * z; }

  constexpr double norm() const { return std::sqrt(norm_squared()); }

  constexpr Vector3 normalized() const {
    double n = norm();
    if (n < 1e-15) {
      throw std::runtime_error("Cannot normalize a zero-length Vector 3");
    }
    return *this / n;
  }

  Vector3 operator+(const Vector3 &o) const {
    return {x + o.x, y + o.y, z + o.z};
  }

  Vector3 operator-(const Vector3 &o) const {
    return {x - o.x, y - o.y, z - o.z};
  }

  Vector3 operator-() const { return {-x, -y, -z}; }

  Vector3 operator*(double s) const { return {x * s, y * s, z * s}; }

  constexpr Vector3 operator/(double s) const {
    if (std::abs(s) < 1e-15) {
      throw std::runtime_error("Division by zero in Vector3 scalar division");
    }
    return {x / s, y / s, z / s};
  }

  Vector3 cross(const Vector3 &o) const {
    return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
  }
};

inline Vector3 operator*(double s, const Vector3 &v) { return v * s; }

inline std::ostream &operator<<(std::ostream &os, const Vector3 &v) {
  os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
  return os;
}
