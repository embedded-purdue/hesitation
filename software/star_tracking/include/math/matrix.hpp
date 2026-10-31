/**
 * @file matrix.hpp
 * @brief 3x3 matrix arithmetic and SO(3) rotation kinematics for star tracking.
 *
 * @details
 * This file provides a 3x3 matrix class for:
 * 1. Direction Cosine Matrices (DCM) transforming vectors between inertial (ECI J2000),
 *    spacecraft body, and camera sensor frames.
 * 2. Intermediate attitude representations in Wahba's problem solvers (Davenport's B matrix,
 *    attitude profile matrix S = B + B^T, and Shuster's QUEST algorithm).
 * 3. Optical transformation models (camera projection, rotation around optical axes).
 *
 * Memory layout is row-major:
 *   [ m00 m01 m02 ]
 *   [ m10 m11 m12 ]
 *   [ m20 m21 m22 ]
 * Stored as index = row * 3 + col.
 *
 * @reference
 * - Shuster, M. D., and Oh, S. D. (1981). "Three-Axis Attitude Determination from Vector
 *   Observations." Journal of Guidance and Control, 4(1), pp. 70–77. DOI: 10.2514/3.19717.
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." JGCD, 36(6), pp. 1672–1680.
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." JGCD, 36(2), pp. 476–488.
 */

#pragma once

#include "math/vector.hpp"
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace star_tracker {

/**
 * @class Matrix3
 * @brief 3x3 real matrix with linear algebra and SO(3) rotation support.
 */
class Matrix3 {
public:
    /// Row-major array storage: m[row * 3 + col]
    std::array<double, 9> m{1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0};

    /// Default constructor initializing to 3x3 identity matrix.
    constexpr Matrix3() = default;

    /**
     * @brief Constructor initializing explicit row-major elements.
     */
    constexpr Matrix3(double m00, double m01, double m02,
                      double m10, double m11, double m12,
                      double m20, double m21, double m22)
        : m{m00, m01, m02,
            m10, m11, m12,
            m20, m21, m22} {}

    /**
     * @brief Returns a 3x3 identity matrix I_3.
     */
    static Matrix3 identity() {
        return Matrix3(1.0, 0.0, 0.0,
                       0.0, 1.0, 0.0,
                       0.0, 0.0, 1.0);
    }

    /**
     * @brief Returns a 3x3 zero matrix 0_{3x3}.
     */
    static Matrix3 zeros() {
        return Matrix3(0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0);
    }

    /**
     * @brief Read-only element access by 0-indexed row and column.
     * @param row Row index [0, 2]
     * @param col Column index [0, 2]
     */
    double operator()(int row, int col) const {
        return m[row * 3 + col];
    }

    /**
     * @brief Mutable element access by 0-indexed row and column.
     * @param row Row index [0, 2]
     * @param col Column index [0, 2]
     */
    double& operator()(int row, int col) {
        return m[row * 3 + col];
    }

    /// Matrix addition: A + B
    Matrix3 operator+(const Matrix3& o) const {
        Matrix3 res;
        for (int i = 0; i < 9; ++i) res.m[i] = m[i] + o.m[i];
        return res;
    }

    /// Matrix subtraction: A - B
    Matrix3 operator-(const Matrix3& o) const {
        Matrix3 res;
        for (int i = 0; i < 9; ++i) res.m[i] = m[i] - o.m[i];
        return res;
    }

    /// Scalar-matrix multiplication: A * s
    Matrix3 operator*(double s) const {
        Matrix3 res;
        for (int i = 0; i < 9; ++i) res.m[i] = m[i] * s;
        return res;
    }

    /**
     * @brief Standard matrix multiplication: C = A * B.
     * Evaluated using 3 nested loops with scalar accumulator.
     */
    Matrix3 operator*(const Matrix3& o) const {
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

    /**
     * @brief Matrix-vector product: y = A * x.
     * Transforms 3D vector x by matrix A.
     */
    Vector3 operator*(const Vector3& v) const {
        return {
            m[0] * v.x + m[1] * v.y + m[2] * v.z,
            m[3] * v.x + m[4] * v.y + m[5] * v.z,
            m[6] * v.x + m[7] * v.y + m[8] * v.z
        };
    }

    /**
     * @brief Computes transpose of matrix: A^T.
     * For orthogonal rotation matrices, A^T = A^-1.
     */
    Matrix3 transpose() const {
        return Matrix3(
            m[0], m[3], m[6],
            m[1], m[4], m[7],
            m[2], m[5], m[8]
        );
    }

    /**
     * @brief Computes scalar determinant det(A) using Laplace cofactor expansion.
     */
    double determinant() const {
        return m[0] * (m[4] * m[8] - m[5] * m[7])
             - m[1] * (m[3] * m[8] - m[5] * m[6])
             + m[2] * (m[3] * m[7] - m[4] * m[6]);
    }

    /**
     * @brief Computes the inverse matrix A^-1 via analytic adjugate cofactor matrix.
     * @throws std::runtime_error if determinant is within numerical zero threshold (|det| < 1e-15).
     */
    Matrix3 inverse() const {
        double det = determinant();
        if (std::abs(det) < 1e-15) {
            throw std::runtime_error("Matrix3 is singular and cannot be inverted");
        }
        double inv_det = 1.0 / det;
        return Matrix3(
            (m[4] * m[8] - m[5] * m[7]) * inv_det,
            (m[2] * m[7] - m[1] * m[8]) * inv_det,
            (m[1] * m[5] - m[2] * m[4]) * inv_det,

            (m[5] * m[6] - m[3] * m[8]) * inv_det,
            (m[0] * m[8] - m[2] * m[6]) * inv_det,
            (m[2] * m[3] - m[0] * m[5]) * inv_det,

            (m[3] * m[7] - m[4] * m[6]) * inv_det,
            (m[1] * m[6] - m[0] * m[7]) * inv_det,
            (m[0] * m[4] - m[1] * m[3]) * inv_det
        );
    }

    /**
     * @brief Generates elementary rotation matrix around X axis by angle theta (radians).
     * @param angle Rotation angle in radians.
     */
    static Matrix3 rot_x(double angle) {
        double c = std::cos(angle);
        double s = std::sin(angle);
        return Matrix3(1.0, 0.0,  0.0,
                       0.0,   c,   -s,
                       0.0,   s,    c);
    }

    /**
     * @brief Generates elementary rotation matrix around Y axis by angle theta (radians).
     * @param angle Rotation angle in radians.
     */
    static Matrix3 rot_y(double angle) {
        double c = std::cos(angle);
        double s = std::sin(angle);
        return Matrix3(  c, 0.0,    s,
                       0.0, 1.0,  0.0,
                        -s, 0.0,    c);
    }

    /**
     * @brief Generates elementary rotation matrix around Z axis (optical boresight) by angle theta (radians).
     * @param angle Rotation angle in radians.
     */
    static Matrix3 rot_z(double angle) {
        double c = std::cos(angle);
        double s = std::sin(angle);
        return Matrix3(  c,  -s,  0.0,
                         s,   c,  0.0,
                       0.0, 0.0,  1.0);
    }

    /**
     * @brief Generates rotation matrix from unit axis and rotation angle using Rodrigues' formula.
     * R(u, theta) = I * cos(theta) + (1 - cos(theta)) * (u * u^T) + [u]_x * sin(theta)
     * @param axis 3D rotation axis (normalized automatically).
     * @param angle Rotation angle in radians.
     */
    static Matrix3 from_axis_angle(const Vector3& axis, double angle) {
        Vector3 u = axis.normalized();
        double c = std::cos(angle);
        double s = std::sin(angle);
        double t = 1.0 - c;

        return Matrix3(
            t * u.x * u.x + c,       t * u.x * u.y - s * u.z, t * u.x * u.z + s * u.y,
            t * u.x * u.y + s * u.z, t * u.y * u.y + c,       t * u.y * u.z - s * u.x,
            t * u.x * u.z - s * u.y, t * u.y * u.z + s * u.x, t * u.z * u.z + c
        );
    }
};

/**
 * @brief Commutative scalar-matrix multiplication operator: s * A.
 */
inline Matrix3 operator*(double s, const Matrix3& mat) {
    return mat * s;
}

} // namespace star_tracker
