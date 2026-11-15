#pragma once

#include "vector.hpp"
#include <array>
#include <cmath>
#include <stdexcept>

class Matrix3 {
public:
  std::array<double, 9> m{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

  constexpr Matrix3() = default;

  constexpr Matrix3(double m00, double m01, double m02, double m10, double m11,
                    double m12, double m20, double m21, double m22)
      : m{m00, m01, m02, m10, m11, m12, m20, m21, m22} {}

  static Matrix3 identity() {
    return Matrix3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
  }

  static Matrix3 zeros() {
    return Matrix3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  static Matrix3 outer(const Vector3 &a, const Vector3 &b) {
    return Matrix3(a.x * b.x, a.x * b.y, a.x * b.z, a.y * b.x, a.y * b.y,
                   a.y * b.z, a.z * b.x, a.z * b.y, a.z * b.z);
  }

  double operator()(int row, int col) const { return m[row * 3 + col]; }

  double &operator()(int row, int col) { return m[row * 3 + col]; }

  Matrix3 operator+(const Matrix3 &o) const {
    Matrix3 res;
    for (int i = 0; i < 9; ++i)
      res.m[i] = m[i] + o.m[i];
    return res;
  }

  Matrix3 operator-(const Matrix3 &o) const {
    Matrix3 res;
    for (int i = 0; i < 9; ++i)
      res.m[i] = m[i] - o.m[i];
    return res;
  }

  Matrix3 operator*(double s) const {
    Matrix3 res;
    for (int i = 0; i < 9; ++i)
      res.m[i] = m[i] * s;
    return res;
  }

  Matrix3 operator*(const Matrix3 &o) const {
    Matrix3 res = zeros();
    for (int i = 0; i < 3; ++i) {
      for (int k = 0; k < 3; ++k) {
        double aik = (*this)(i, k);
        for (int j = 0; j < 3; ++j) {
          res(i, j) += aik * o(k, j);
        }
      }
    }
    return res;
  }

  Vector3 operator*(const Vector3 &v) const {
    return {m[0] * v.x + m[1] * v.y + m[2] * v.z,
            m[3] * v.x + m[4] * v.y + m[5] * v.z,
            m[6] * v.x + m[7] * v.y + m[8] * v.z};
  }

  Matrix3 transpose() const {
    return Matrix3(m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]);
  }

  double determinant() const {
    return m[0] * (m[4] * m[8] - m[5] * m[7]) -
           m[1] * (m[3] * m[8] - m[5] * m[6]) +
           m[2] * (m[3] * m[7] - m[4] * m[6]);
  }

  Matrix3 inverse() const {
    double det = determinant();
    if (std::abs(det) < 1e-15) {
      throw std::runtime_error("Matrix3 is singular and cannot be inverted");
    }
    double inv_det = 1.0 / det;
    return Matrix3((m[4] * m[8] - m[5] * m[7]) * inv_det,
                   (m[2] * m[7] - m[1] * m[8]) * inv_det,
                   (m[1] * m[5] - m[2] * m[4]) * inv_det,

                   (m[5] * m[6] - m[3] * m[8]) * inv_det,
                   (m[0] * m[8] - m[2] * m[6]) * inv_det,
                   (m[2] * m[3] - m[0] * m[5]) * inv_det,

                   (m[3] * m[7] - m[4] * m[6]) * inv_det,
                   (m[1] * m[6] - m[0] * m[7]) * inv_det,
                   (m[0] * m[4] - m[1] * m[3]) * inv_det);
  }
};

inline Matrix3 operator*(double s, const Matrix3 &mat) { return mat * s; }
