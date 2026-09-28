#pragma once
#include "vec3.hpp"
#include <algorithm>
#include <cmath>

struct Quat {
  double x, y, z, w;

  Quat() : x(0), y(0), z(0), w(1) {}
  Quat(double x, double y, double z, double w) : x(x), y(y), z(z), w(w) {}
  Quat(const Vec3 &v, double s) : x(v.x), y(v.y), z(v.z), w(s) {}

  Quat operator+(const Quat &o) const {
    return Quat(x + o.x, y + o.y, z + o.z, w + o.w);
  }
  Quat operator*(double s) const { return Quat(x * s, y * s, z * s, w * s); }

  Vec3 vec() const { return Vec3(x, y, z); }

  double norm() const { return std::sqrt(x * x + y * y + z * z + w * w); }

  Quat normalized() const { return *this * (1.0 / norm); }

  // calculates rotation angle of the quanternion
  double angle() const {
    // w = cos( phi / 2 )
    // phi = 2cos^-1(w)
    return 2.0 * std::acos(std::abs(std::clamp(w, -1.0, 1.0))); // eq. 10
  }

  // constructs the quanternion given axis and angle
  static Quat from_axis_angle(const Vec3 &axis, double phi) {
    return Quat(axis * std::sin(phi / 2.0),
                std::cos(phi / 2.0)); // refer to equation 10
  }
};

// allows notation to be reversed: "2.0 * q" as well as "q * 2.0"
inline Quat operator*(double s, const Quat &q) { return q * s; }

// calculates the omega mull, refer to eq. 11
inline Quat omega_mul(const Vec3 &w, const Quat &q) {
  const Vec3 qv = q.vec();
  return Quat(w * q.w - w.cross(qv), -w.dot(qv));
}
