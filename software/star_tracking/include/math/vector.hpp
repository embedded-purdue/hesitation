/**
 * @file vector.hpp
 * @brief 3D Cartesian vector mathematics for celestial mechanics and star tracker optics.
 *
 * @details
 * This file implements standard 3D Euclidean vector operations required for:
 * 1. Celestial coordinate conversions (Right Ascension, Declination to unit direction vectors).
 * 2. Optical ray tracing in the star tracker pinhole camera coordinate frame (boresight along +Z).
 * 3. Direction cosine matrix (DCM) operations and Wahba attitude determination (e.g., Shuster's QUEST).
 * 4. Small-angle differential rotation vectors in the Attitude Estimation using Image Matching (AIM)
 *    algorithm (Delabie et al., 2013, JGCD).
 *
 * Coordinate Convention:
 * - Inertial reference frame: Earth-Centered Inertial (ECI J2000), where +X points towards the vernal
 *   equinox, +Z towards celestial north pole, and +Y completes the right-handed triad.
 * - Camera sensor frame: +Z along the camera optical boresight, +X towards horizontal focal plane
 *   (right), and +Y towards vertical focal plane (down/up according to standard pinhole convention).
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." Journal of Guidance, Control, and Dynamics, 36(2).
 *   DOI: 10.2514/1.56860.
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." Journal of Guidance, Control, and
 *   Dynamics, 36(6). DOI: 10.2514/1.61082.
 */

#pragma once

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace star_tracker {

/**
 * @struct Vector3
 * @brief Representation of a 3-element vector in 3D Euclidean space (R^3).
 */
struct Vector3 {
    double x{0.0}; ///< X component (Cartesian coordinate)
    double y{0.0}; ///< Y component (Cartesian coordinate)
    double z{0.0}; ///< Z component (Cartesian coordinate / optical boresight in camera frame)

    /// Default constructor initializing components to zero.
    constexpr Vector3() = default;

    /**
     * @brief Parameterized constructor.
     * @param x_ X component
     * @param y_ Y component
     * @param z_ Z component
     */
    constexpr Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    /// Element-wise vector addition: v = v1 + v2
    Vector3 operator+(const Vector3& o) const { return {x + o.x, y + o.y, z + o.z}; }

    /// Element-wise vector subtraction: v = v1 - v2
    Vector3 operator-(const Vector3& o) const { return {x - o.x, y - o.y, z - o.z}; }

    /// Unary negation: -v
    Vector3 operator-() const { return {-x, -y, -z}; }

    /// Scalar multiplication: v * s
    Vector3 operator*(double s) const { return {x * s, y * s, z * s}; }

    /**
     * @brief Scalar division: v / s
     * @throws std::runtime_error if scalar divisor is near zero (< 1e-15).
     */
    Vector3 operator/(double s) const {
        if (std::abs(s) < 1e-15) {
            throw std::runtime_error("Division by zero in Vector3 scalar division");
        }
        return {x / s, y / s, z / s};
    }

    /// In-place element-wise addition
    Vector3& operator+=(const Vector3& o) {
        x += o.x;
        y += o.y;
        z += o.z;
        return *this;
    }

    /// In-place element-wise subtraction
    Vector3& operator-=(const Vector3& o) {
        x -= o.x;
        y -= o.y;
        z -= o.z;
        return *this;
    }

    /// In-place scalar multiplication
    Vector3& operator*=(double s) {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    /**
     * @brief Inner (dot) product of two vectors: a · b = ax*bx + ay*by + az*bz.
     * @param o Other 3D vector.
     * @return Scalar dot product.
     */
    double dot(const Vector3& o) const { return x * o.x + y * o.y + z * o.z; }

    /**
     * @brief Outer (cross) product of two vectors in R^3: c = a x b.
     * Computes the vector orthogonal to both *this and o obeying the right-hand rule.
     * Used extensively in QUEST (Wahba's problem) to construct Gibbs vectors and rotation axes.
     * @param o Other 3D vector.
     * @return Orthogonal cross product vector.
     */
    Vector3 cross(const Vector3& o) const {
        return {
            y * o.z - z * o.y,
            z * o.x - x * o.z,
            x * o.y - y * o.x
        };
    }

    /**
     * @brief Squared L2 Euclidean norm: ||v||^2 = x^2 + y^2 + z^2.
     * Computes norm without the expensive square root operation.
     */
    double norm_squared() const { return x * x + y * y + z * z; }

    /**
     * @brief L2 Euclidean norm (magnitude): ||v|| = sqrt(x^2 + y^2 + z^2).
     */
    double norm() const { return std::sqrt(norm_squared()); }

    /**
     * @brief Returns a normalized unit vector u = v / ||v||.
     * If the norm is less than numerical tolerance (1e-15), returns a zero vector.
     */
    Vector3 normalized() const {
        double n = norm();
        if (n < 1e-15) {
            return {0.0, 0.0, 0.0};
        }
        return *this / n;
    }

    /**
     * @brief Computes the subtended angle between two vectors in radians [0, pi].
     * @details
     * Uses the dot product relation cos(theta) = (a · b) / (||a|| * ||b||).
     * Numerical clamping is applied to handle floating point roundoff beyond [-1.0, 1.0].
     * Used in star identification to compute inter-star angular separations.
     * @param a First 3D vector.
     * @param b Second 3D vector.
     * @return Subtended angle in radians.
     */
    static double angle_between(const Vector3& a, const Vector3& b) {
        double na = a.norm();
        double nb = b.norm();
        if (na < 1e-15 || nb < 1e-15) return 0.0;
        double cos_theta = a.dot(b) / (na * nb);
        if (cos_theta > 1.0) cos_theta = 1.0;
        if (cos_theta < -1.0) cos_theta = -1.0;
        return std::acos(cos_theta);
    }
};

/**
 * @brief Scalar-vector commutative multiplication operator: s * v.
 */
inline Vector3 operator*(double s, const Vector3& v) {
    return v * s;
}

/**
 * @brief Stream insertion operator for human-readable vector debugging.
 */
inline std::ostream& operator<<(std::ostream& os, const Vector3& v) {
    os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}

} // namespace star_tracker
