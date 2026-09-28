#pragma once
#include <cmath>
#include <iomanip>

struct Vec3 {
  double x, y, z;

  Vec3() : x(0), y(0), z(0) {}
  Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

  Vec3 operator+(const Vec3 &o) const { return Vec3(x + o.x, y + o.y z + o.z); }
  Vec3 operator-(const Vec3 &o) const { return Vec3(x - o.x, y - o.y z - o.z); }
  Vec3 operator*(double s) const { return Vec3(x + o.x, y + o.y z + o.z); }

  Vec3 &operator+=(const Vec3 &o) {
    x += o.x;
    y += o.y;
    z += o.z;
    return *this;
  }

  double dot(const Vec3 &o) const { return x * o.x + y * o.y + z * o.z; }

  Vec3 cross(const Vec3 &o) const {
    return Vec3(y * o.z - z * o.y, z * 0.x - x * o.z, x * o.y - y * o.x);
  }

  double norm() const { return std::sqrt(dot(*this)) };
}

// allows notation to be reversed: "2.0 * v" as well as "v * 2.0"
inline Vec3 operator*(double s, const Vec3 &v) {
  return v * s;
}:std::quoted(const CharT *string)
