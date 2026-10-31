/**
 * @file quaternion.hpp
 * @brief Hamilton unit quaternions for 3D spacecraft attitude representations and kinematics.
 *
 * @details
 * This file implements unit quaternions (Euler-Rodrigues symmetric parameters) following the
 * standard Hamilton convention:
 *   q = w + x*i + y*j + z*k = [w, v^T]^T
 * where w is the scalar component and (x, y, z) is the imaginary vector component, with:
 *   i^2 = j^2 = k^2 = i*j*k = -1
 *
 * Key Capabilities for Star Tracking:
 * 1. Attitude Representation: Represents the rotation from inertial frame (ECI J2000) to
 *    camera sensor frame (or body frame).
 * 2. Vector Rotation: Transforms celestial star vectors v via:
 *      v_cam = R(q) * v_inertial = q * [0, v]^T * q^-1
 *    (See Delabie et al., 2013, "AIM", Eq. 5).
 * 3. Composition: Successive rotations are computed via the Hamilton quaternion product:
 *      q_total = q_second * q_first
 *    Used in AIM differential updates: q_e = q_diff * q_dat (Delabie et al., 2013, "AIM", Eq. 21).
 * 4. Small-Angle Differential Update: Small rotational error vector delta = [theta_x, theta_y, theta_z]^T:
 *      q_diff ≈ [1, 0.5*theta_x, 0.5*theta_y, 0.5*theta_z]^T (Delabie et al., 2013, "AIM", Eq. 20).
 * 5. Geodesic Angular Distance: Computes exact spatial angular misalignment between attitudes:
 *      Phi = 2 * arccos(|q1 · q2|)
 *
 * @reference
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." Journal of Guidance, Control,
 *   and Dynamics, 36(6), pp. 1672–1680. DOI: 10.2514/1.61082.
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." Journal of Guidance, Control, and Dynamics, 36(2),
 *   pp. 476–488. DOI: 10.2514/1.56860.
 * - Shuster, M. D. (1993). "A Survey of Attitude Representations." Journal of the Astronautical
 *   Sciences, 41(4), pp. 439–517.
 */

#pragma once

#include "math/matrix.hpp"
#include "math/vector.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace star_tracker {

/**
 * @struct Quaternion
 * @brief Unit quaternion for singularity-free 3D attitude representation.
 */
struct Quaternion {
    double w{1.0}; ///< Scalar (real) part: cos(theta / 2)
    double x{0.0}; ///< Vector part X: u_x * sin(theta / 2)
    double y{0.0}; ///< Vector part Y: u_y * sin(theta / 2)
    double z{0.0}; ///< Vector part Z: u_z * sin(theta / 2)

    /// Default constructor initializing to identity quaternion [1, 0, 0, 0] (zero rotation).
    constexpr Quaternion() = default;

    /**
     * @brief Explicit component constructor.
     * @param w_ Scalar component
     * @param x_ Vector component X
     * @param y_ Vector component Y
     * @param z_ Vector component Z
     */
    constexpr Quaternion(double w_, double x_, double y_, double z_)
        : w(w_), x(x_), y(y_), z(z_) {}

    /**
     * @brief Returns the identity quaternion representing zero rotation [1, 0, 0, 0].
     */
    static Quaternion identity() {
        return Quaternion(1.0, 0.0, 0.0, 0.0);
    }

    /**
     * @brief Constructs a unit quaternion from an axis and rotation angle.
     * @details
     * q = [cos(theta/2), u * sin(theta/2)]
     * @param axis 3D rotation axis vector (normalized automatically).
     * @param angle Rotation angle in radians.
     */
    static Quaternion from_axis_angle(const Vector3& axis, double angle) {
        Vector3 u = axis.normalized();
        double half_angle = 0.5 * angle;
        double s = std::sin(half_angle);
        return Quaternion(std::cos(half_angle), u.x * s, u.y * s, u.z * s).normalized();
    }

    /**
     * @brief Constructs a quaternion from an unnormalized 3D rotation vector r = theta * u.
     * @details
     * For small rotation vectors (theta < 1e-12), uses the first-order Taylor expansion:
     *   q ≈ [1, 0.5 * r_x, 0.5 * r_y, 0.5 * r_z]
     * matching the small-angle approximation in Delabie et al. (2013) Eq. (20).
     * @param r 3D rotation vector in radians.
     */
    static Quaternion from_rotation_vector(const Vector3& r) {
        double theta = r.norm();
        if (theta < 1e-12) {
            return Quaternion(1.0, 0.5 * r.x, 0.5 * r.y, 0.5 * r.z).normalized();
        }
        return from_axis_angle(r, theta);
    }

    /**
     * @brief Constructs a quaternion from Tait-Bryan Euler angles in 3-2-1 sequence.
     * @details
     * Rotation sequence: R = Rz(yaw) * Ry(pitch) * Rx(roll).
     * @param roll_x Roll angle around X axis (radians).
     * @param pitch_y Pitch angle around Y axis (radians).
     * @param yaw_z Yaw angle around Z axis (radians).
     */
    static Quaternion from_euler(double roll_x, double pitch_y, double yaw_z) {
        double cr = std::cos(roll_x * 0.5);
        double sr = std::sin(roll_x * 0.5);
        double cp = std::cos(pitch_y * 0.5);
        double sp = std::sin(pitch_y * 0.5);
        double cy = std::cos(yaw_z * 0.5);
        double sy = std::sin(yaw_z * 0.5);

        return Quaternion(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        ).normalized();
    }

    /**
     * @brief Extracts attitude quaternion from a 3x3 Direction Cosine Matrix (DCM).
     * @details
     * Uses Shepperd's algorithm for numerical stability, selecting the maximum diagonal
     * element to avoid division by near-zero values.
     * @param R Orthogonal 3x3 rotation matrix.
     */
    static Quaternion from_matrix(const Matrix3& R) {
        double tr = R(0, 0) + R(1, 1) + R(2, 2);
        Quaternion q;
        if (tr > 0.0) {
            double s = 0.5 / std::sqrt(tr + 1.0);
            q.w = 0.25 / s;
            q.x = (R(2, 1) - R(1, 2)) * s;
            q.y = (R(0, 2) - R(2, 0)) * s;
            q.z = (R(1, 0) - R(0, 1)) * s;
        } else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
            double s = 2.0 * std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2));
            q.w = (R(2, 1) - R(1, 2)) / s;
            q.x = 0.25 * s;
            q.y = (R(0, 1) + R(1, 0)) / s;
            q.z = (R(0, 2) + R(2, 0)) / s;
        } else if (R(1, 1) > R(2, 2)) {
            double s = 2.0 * std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2));
            q.w = (R(0, 2) - R(2, 0)) / s;
            q.x = (R(0, 1) + R(1, 0)) / s;
            q.y = 0.25 * s;
            q.z = (R(1, 2) + R(2, 1)) / s;
        } else {
            double s = 2.0 * std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1));
            q.w = (R(1, 0) - R(0, 1)) / s;
            q.x = (R(0, 2) + R(2, 0)) / s;
            q.y = (R(1, 2) + R(2, 1)) / s;
            q.z = 0.25 * s;
        }
        return q.normalized();
    }

    /// Squared quaternion Euclidean norm: ||q||^2 = w^2 + x^2 + y^2 + z^2.
    double norm_squared() const {
        return w * w + x * x + y * y + z * z;
    }

    /// Quaternion Euclidean norm: ||q|| = sqrt(w^2 + x^2 + y^2 + z^2).
    double norm() const {
        return std::sqrt(norm_squared());
    }

    /**
     * @brief Normalizes quaternion to unit length (||q|| = 1).
     * If norm < 1e-15, returns identity quaternion.
     */
    Quaternion normalized() const {
        double n = norm();
        if (n < 1e-15) {
            return identity();
        }
        double inv = 1.0 / n;
        return Quaternion(w * inv, x * inv, y * inv, z * inv);
    }

    /**
     * @brief Quaternion conjugate q* = [w, -x, -y, -z].
     * For unit quaternions, q* is equal to the inverse q^-1.
     * @see Delabie et al. (2013) AIM Eq. (5).
     */
    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    /**
     * @brief Computes multiplicative inverse q^-1 = q* / ||q||^2.
     */
    Quaternion inverse() const {
        double n2 = norm_squared();
        if (n2 < 1e-15) {
            return identity();
        }
        double inv = 1.0 / n2;
        return Quaternion(w * inv, -x * inv, -y * inv, -z * inv);
    }

    /**
     * @brief Hamilton quaternion multiplication: q_res = q_1 ⊗ q_2.
     * @details
     * Computes the compound rotation corresponding to rotating first by o and then by *this.
     * @see Delabie et al. (2013) AIM Eq. (21): q_e = q_diff ⊗ q_dat.
     */
    Quaternion operator*(const Quaternion& o) const {
        return Quaternion(
            w * o.w - x * o.x - y * o.y - z * o.z,
            w * o.x + x * o.w + y * o.z - z * o.y,
            w * o.y - x * o.z + y * o.w + z * o.x,
            w * o.z + x * o.y - y * o.x + z * o.w
        );
    }

    /// Element-wise quaternion addition
    Quaternion operator+(const Quaternion& o) const {
        return {w + o.w, x + o.x, y + o.y, z + o.z};
    }

    /// Element-wise quaternion subtraction
    Quaternion operator-(const Quaternion& o) const {
        return {w - o.w, x - o.x, y - o.y, z - o.z};
    }

    /// Scalar-quaternion multiplication
    Quaternion operator*(double s) const {
        return {w * s, x * s, y * s, z * s};
    }

    /// 4D inner product: q1 · q2 = w1*w2 + x1*x2 + y1*y2 + z1*z2
    double dot(const Quaternion& o) const {
        return w * o.w + x * o.x + y * o.y + z * o.z;
    }

    /**
     * @brief Converts unit quaternion to an equivalent 3x3 Direction Cosine Matrix (DCM).
     * @details
     * Transforms vectors such that: v' = R * v.
     */
    Matrix3 to_matrix() const {
        double x2 = x * x, y2 = y * y, z2 = z * z;
        return Matrix3(
            1.0 - 2.0 * (y2 + z2), 2.0 * (x * y - w * z),       2.0 * (x * z + w * y),
            2.0 * (x * y + w * z), 1.0 - 2.0 * (x2 + z2),       2.0 * (y * z - w * x),
            2.0 * (x * z - w * y), 2.0 * (y * z + w * x),       1.0 - 2.0 * (x2 + y2)
        );
    }

    /**
     * @brief Rotates a 3D vector v by this unit quaternion: v' = q ⊗ [0, v] ⊗ q^-1.
     * Evaluated using DCM multiplication for computational efficiency.
     * @param v 3D vector in source frame.
     * @return Rotated 3D vector in target frame.
     */
    Vector3 rotate(const Vector3& v) const {
        return to_matrix() * v;
    }

    /**
     * @brief Computes geodesic spatial angular distance (radians) between two unit quaternions.
     * @details
     * Accounts for the double-cover property of SO(3) by taking the absolute value of the dot product:
     *   theta = 2 * arccos(|q1 · q2|)
     * Result is in radians in range [0, pi].
     * @param q1 First attitude quaternion.
     * @param q2 Second attitude quaternion.
     * @return Angular error in radians.
     */
    static double angular_distance(const Quaternion& q1, const Quaternion& q2) {
        double d = std::abs(q1.dot(q2));
        if (d > 1.0) d = 1.0;
        return 2.0 * std::acos(d);
    }
};

/**
 * @brief Commutative scalar-quaternion multiplication operator: s * q.
 */
inline Quaternion operator*(double s, const Quaternion& q) {
    return q * s;
}

/**
 * @brief Stream insertion operator for human-readable quaternion logging.
 */
inline std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
    os << "[" << q.w << ", " << q.x << ", " << q.y << ", " << q.z << "]";
    return os;
}

} // namespace star_tracker
