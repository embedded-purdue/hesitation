/**
 * @file quat.hpp
 * @brief Unit quaternion representation and kinematic operations for rigid-body attitude.
 *
 * Implements unit quaternion operations following the JPL/paper conventions where:
 *   q = [q_v, q_s]^T = [x, y, z, w]^T
 *   q_s = w = cos(phi / 2)
 *   q_v = [x, y, z]^T = u * sin(phi / 2)
 *
 * Kinematic differential equation:
 *   qdot = 0.5 * Omega(w) * q  (Elkins et al., Eq. 11)
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
#include "vec3.hpp"
#include <algorithm>
#include <cmath>

/**
 * @brief Quaternion struct representing 3D rotational attitude.
 *
 * Components:
 *   x, y, z: Vector part (axis of rotation scaled by sin(phi/2))
 *   w:       Scalar part (cos(phi/2))
 */
struct Quat {
  double x, y, z, w;

  /// Default constructor initializing to identity rotation [0, 0, 0, 1].
  Quat() : x(0), y(0), z(0), w(1) {}

  /// Component constructor (x, y, z, w).
  Quat(double x, double y, double z, double w) : x(x), y(y), z(z), w(w) {}

  /// Vector part + scalar part constructor.
  Quat(const Vec3 &v, double s) : x(v.x), y(v.y), z(v.z), w(s) {}

  /// Quaternion addition.
  Quat operator+(const Quat &o) const {
    return Quat(x + o.x, y + o.y, z + o.z, w + o.w);
  }

  /// Scalar multiplication.
  Quat operator*(double s) const { return Quat(x * s, y * s, z * s, w * s); }

  /// Vector part extraction: [x, y, z]^T.
  Vec3 vec() const { return Vec3(x, y, z); }

  /// Euclidean 4-norm of the quaternion.
  double norm() const { return std::sqrt(x * x + y * y + z * z + w * w); }

  /// Normalized unit quaternion.
  Quat normalized() const { return *this * (1.0 / norm()); }

  /**
   * @brief Calculates the principal rotation angle phi in radians.
   *
   * w = cos(phi / 2)  =>  phi = 2 * acos(|w|)
   * Refer to Elkins et al., Eq. (10).
   *
   * @return double Principal angle in radians [0, pi].
   */
  double angle() const {
    return 2.0 * std::acos(std::abs(std::clamp(w, -1.0, 1.0))); // eq. 10
  }

  /**
   * @brief Constructs a unit quaternion from an axis of rotation and angle phi.
   *
   * q = [axis * sin(phi / 2), cos(phi / 2)]^T
   * Refer to Elkins et al., Eq. (10).
   *
   * @param axis Unit vector representing the axis of rotation.
   * @param phi  Rotation angle in radians.
   * @return Quat Constructed unit quaternion.
   */
  static Quat from_axis_angle(const Vec3 &axis, double phi) {
    return Quat(axis * std::sin(phi / 2.0),
                std::cos(phi / 2.0)); // refer to equation 10
  }
};

/// Scalar multiplication operator allowing commutative notation (e.g., s * q).
inline Quat operator*(double s, const Quat &q) { return q * s; }

/**
 * @brief Computes quaternion rate product Omega(w) * q.
 *
 * Evaluates the quaternion kinematic product:
 *   qdot = 0.5 * Omega(w) * q
 * where Omega(w) is the skew-symmetric matrix representation of angular velocity w.
 * Refer to Elkins et al., Eq. (11).
 *
 * @param w Body angular velocity vector [rad/s].
 * @param q Current attitude unit quaternion.
 * @return Quat 2 * qdot (multiply by 0.5 to get time derivative qdot).
 */
inline Quat omega_mul(const Vec3 &w, const Quat &q) {
  const Vec3 qv = q.vec();
  return Quat(w * q.w - w.cross(qv), -w.dot(qv));
}
