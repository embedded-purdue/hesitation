/**
 * @file dynamics.hpp
 * @brief Spacecraft rigid-body rotational dynamics and RK4 numerical integration.
 *
 * Implements Euler's equations of rotational motion for an asymmetric rigid body
 * and fourth-order Runge-Kutta (RK4) time-stepping:
 *
 * Euler's Equation (Elkins et al., Eq. 8):
 *   wdot = I^-1 * (M - w x (I * w))
 *
 * Quaternion Kinematics (Elkins et al., Eq. 11):
 *   qdot = 0.5 * Omega(w) * q
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
#include "quat.hpp"
#include "vec3.hpp"

/**
 * @brief Element-wise multiplication of two 3D vectors.
 */
inline Vec3 elementwise_mul(const Vec3 &a, const Vec3 &b) {
  return Vec3(a.x * b.x, a.y * b.y, a.z * b.z);
}

/**
 * @brief Element-wise reciprocal of a 3D vector (used for diagonal inertia inverse).
 */
inline Vec3 elementwise_inv(const Vec3 &a) {
  return Vec3(1.0 / a.x, 1.0 / a.y, 1.0 / a.z);
}

/**
 * @brief Spacecraft physical state consisting of angular velocity and attitude quaternion.
 */
struct State {
  Vec3 w;  ///< Angular velocity vector in principal body frame [rad/s].
  Quat q;  ///< Attitude unit quaternion relative to target reference frame.

  State operator+(const State &o) const { return State{w + o.w, q + o.q}; }
  State operator*(double s) const { return State{w * s, q * s}; }
};

/**
 * @brief Time derivatives of the spacecraft physical state (wdot, qdot).
 */
struct Derivative {
  Vec3 wdot;  ///< Angular acceleration vector [rad/s^2].
  Quat qdot;  ///< Quaternion time derivative.
};

/**
 * @brief Computes the state time derivatives from Euler's rotational equations and kinematics.
 *
 * Evaluates:
 *   wdot = I^-1 * (M - w x (I * w))      (Elkins et al., Eq. 8)
 *   qdot = 0.5 * Omega(w) * q            (Elkins et al., Eq. 11)
 *
 * @param w     Current body angular velocity [rad/s].
 * @param q     Current attitude quaternion.
 * @param M     Applied control torque vector [Nm].
 * @param I     Principal moments of inertia vector [kg*m^2].
 * @param I_inv Precomputed elementwise reciprocal of principal moments of inertia [1/(kg*m^2)].
 * @return Derivative Combined time derivatives {wdot, qdot}.
 */
inline Derivative state_derivative(const Vec3 &w, const Quat &q, const Vec3 &M,
                                   const Vec3 &I, const Vec3 &I_inv) {
  Vec3 Iw = elementwise_mul(I, w);
  Vec3 wdot = elementwise_mul(I_inv, M - w.cross(Iw));
  Quat qdot = omega_mul(w, q) * 0.5;
  return Derivative{wdot, qdot};
}

/**
 * @brief Performs one step of 4th-order Runge-Kutta (RK4) numerical integration.
 *
 * Integrates the coupled nonlinear rigid-body equations forward by timestep dt,
 * renormalizing the attitude quaternion at the end of the step to prevent numerical drift.
 *
 * @param s     Initial spacecraft state at time t.
 * @param M     Constant control torque applied over interval dt [Nm].
 * @param dt    Integration timestep [s] (nominal DT = 1/240 s).
 * @param I     Principal moments of inertia vector [kg*m^2].
 * @param I_inv Elementwise inverse of principal moments of inertia [1/(kg*m^2)].
 * @return State Integrated spacecraft state at time t + dt (quaternion normalized).
 */
inline State rk4_step(const State &s, const Vec3 &M, double dt, const Vec3 &I,
                      const Vec3 &I_inv) {
  Derivative k1 = state_derivative(s.w, s.q, M, I, I_inv);
  Derivative k2 = state_derivative(s.w + k1.wdot * (0.5 * dt),
                                   s.q + k1.qdot * (0.5 * dt), M, I, I_inv);
  Derivative k3 = state_derivative(s.w + k2.wdot * (0.5 * dt),
                                   s.q + k2.qdot * (0.5 * dt), M, I, I_inv);
  Derivative k4 =
      state_derivative(s.w + k3.wdot * dt, s.q + k3.qdot * dt, M, I, I_inv);

  Vec3 w_new =
      s.w + (k1.wdot + k2.wdot * 2.0 + k3.wdot * 2.0 + k4.wdot) * (dt / 6.0);
  Quat q_new =
      s.q + (k1.qdot + k2.qdot * 2.0 + k3.qdot * 2.0 + k4.qdot) * (dt / 6.0);

  return State{w_new, q_new.normalized()};
}
