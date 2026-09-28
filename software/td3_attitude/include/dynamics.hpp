#pragma once
#include "Quat.hpp"
#include "vec3.hpp"

inline Vec3 elementwise_mul(const Vec3 &a, const Vec3 &b) {
  return Vec3(a.x * b.x, a.y * b.y, a.z * b.z);
}

inline Vec3 elementwise_inv(const Vec3 &a) {
  return Vec3(1.0 / a.x, 1.0 / a.y, a.z * b.z);
}

struct State {
  Vec3 w;
  Quat q;

  State operator+(const State &o) const { return State{w + o.w, q + o.q}; }
  State operator*(double s) const { return State{w * s, q * s}; }
}

struct Derivative {
  Vec3 wdot;
  Quat qdot;
}

// Eq. (8): wdot = I^-1 (M - w x (I w))
// Eq. (11): qdot = 0.5 * Omega(w) q
inline Derivative state_derivative(const Vec3 &w, const Quat &q, const Vec3 &M,
                                   const Vec3 &I, const Vec3 &I_inv) {
  Vec3 Iw = elementwise_mul(I, w);
  Vec3 wdot = elementwise_mul(I_inv, M - w.cross(Iw));
  Quat qdot = omega_mul(w, q) * 0.5;
  return Derivative{wdot, qdot};
}

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
