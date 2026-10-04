/**
 * @file vec3.hpp
 * @brief 3D vector arithmetic and vector algebra utilities.
 *
 * Provides basic vector operations including dot product, cross product,
 * elementwise operations, Euclidean norm, and scalar multiplication
 * used throughout spacecraft rigid-body dynamics and attitude calculations.
 *
 * @author Eric Yu
 * @date October 2026
 * @version 1.0.0
 *
 * Reference:
 *   Elkins et al., "Spacecraft Attitude Control Using Deep Reinforcement Learning,"
 *   AAS 20-475.
 */

#pragma once
#include <cmath>

/**
 * @brief Three-dimensional Euclidean vector struct.
 */
struct Vec3 {
  double x, y, z;

  /// Default constructor initializing to zero vector.
  Vec3() : x(0), y(0), z(0) {}

  /// Component constructor.
  Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

  /// Vector addition.
  Vec3 operator+(const Vec3 &o) const {
    return Vec3(x + o.x, y + o.y, z + o.z);
  }

  /// Vector subtraction.
  Vec3 operator-(const Vec3 &o) const {
    return Vec3(x - o.x, y - o.y, z - o.z);
  }

  /// Scalar multiplication.
  Vec3 operator*(double s) const { return Vec3(x * s, y * s, z * s); }

  /// In-place vector addition.
  Vec3 &operator+=(const Vec3 &o) {
    x += o.x;
    y += o.y;
    z += o.z;
    return *this;
  }

  /// Vector dot product: x1*x2 + y1*y2 + z1*z2.
  double dot(const Vec3 &o) const { return x * o.x + y * o.y + z * o.z; }

  /// Vector cross product: self x other.
  Vec3 cross(const Vec3 &o) const {
    return Vec3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x);
  }

  /// Euclidean L2 norm of the vector: sqrt(x^2 + y^2 + z^2).
  double norm() const { return std::sqrt(dot(*this)); }
};

/// Scalar multiplication operator allowing commutative notation (e.g., s * v).
inline Vec3 operator*(double s, const Vec3 &v) { return v * s; }
