/**
 * @file distortion.hpp
 * @brief Optical lens distortion models for star tracker camera calibration.
 *
 * @details
 * In star tracker optical design, wide field-of-view lenses (typically 10 to 30 degrees)
 * introduce non-negligible geometric distortion. If uncorrected, this distortion shifts star
 * centroids on the focal plane by several pixels, introducing attitude errors of hundreds
 * of arcseconds (Delabie et al., 2013).
 *
 * This file implements radial lens distortion up to the 5th order:
 *   r^2 = x_n^2 + y_n^2
 *   radial_scale = 1 + k_1 * r^2 + k_2 * r^4
 *   x_d = x_n * radial_scale
 *   y_d = y_n * radial_scale
 * where:
 *   - (x_n, y_n): Undistorted normalized coordinates.
 *   - (x_d, y_d): Distorted normalized coordinates as measured on the physical sensor.
 *   - k_1: 3rd-order radial distortion coefficient.
 *   - k_2: 5th-order radial distortion coefficient.
 *
 * Inverse mapping (from distorted to undistorted coordinates) is computed using fixed-point
 * numerical iteration, converging in < 5 iterations to sub-micro-pixel precision.
 *
 * In the AIM paper (Delabie et al., 2013, Section II.A & II.C, Eqs. 3-8), it is noted that
 * applying distortion to database stars rather than undistorting observed stars allows
 * a huge increase in computational efficiency because the conversion is performed on the
 * pre-selected database stars.
 *
 * @reference
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." JGCD, 36(6), pp. 1672–1680.
 *   Section II.A "Sequence of Algorithms" and Section II.C "Database Stars Coordinate Conversion".
 *   DOI: 10.2514/1.61082.
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." JGCD, 36(2), pp. 476–488. DOI: 10.2514/1.56860.
 */

#pragma once

#include <cmath>

namespace star_tracker {

/**
 * @struct DistortionModel
 * @brief 5th-order polynomial radial distortion model.
 */
struct DistortionModel {
    double k1{0.0}; ///< 3rd-order radial coefficient (applied to r^2)
    double k2{0.0}; ///< 5th-order radial coefficient (applied to r^4)

    /// Default constructor representing an ideal, distortion-free pinhole lens.
    constexpr DistortionModel() = default;

    /**
     * @brief Parameterized constructor for lens calibration parameters.
     * @param k1_ 3rd-order radial distortion coefficient.
     * @param k2_ 5th-order radial distortion coefficient.
     */
    constexpr DistortionModel(double k1_, double k2_) : k1(k1_), k2(k2_) {}

    /**
     * @brief Checks if the lens distortion is numerically zero (ideal pinhole).
     */
    bool is_identity() const {
        return std::abs(k1) < 1e-12 && std::abs(k2) < 1e-12;
    }

    /**
     * @brief Applies forward lens distortion to undistorted normalized coordinates.
     * @details
     * Maps ideal pinhole coordinates (xn, yn) to physically distorted coordinates (xd, yd):
     *   r^2 = xn^2 + yn^2
     *   xd = xn * (1 + k1 * r^2 + k2 * r^4)
     *   yd = yn * (1 + k1 * r^2 + k2 * r^4)
     *
     * @param xn Undistorted normalized X coordinate.
     * @param yn Undistorted normalized Y coordinate.
     * @param[out] xd Distorted normalized X coordinate.
     * @param[out] yd Distorted normalized Y coordinate.
     */
    void apply(double xn, double yn, double& xd, double& yd) const {
        if (is_identity()) {
            xd = xn;
            yd = yn;
            return;
        }
        double r2 = xn * xn + yn * yn;
        double radial = 1.0 + k1 * r2 + k2 * r2 * r2;
        xd = xn * radial;
        yd = yn * radial;
    }

    /**
     * @brief Inverts lens distortion to recover undistorted normalized coordinates.
     * @details
     * Given distorted coordinates (xd, yd) measured by the sensor, computes undistorted (xn, yn)
     * via fixed-point iteration:
     *   xn_{k+1} = xd / (1 + k1 * r_k^2 + k2 * r_k^4)
     *   yn_{k+1} = yd / (1 + k1 * r_k^2 + k2 * r_k^4)
     *
     * @param xd Distorted normalized X coordinate.
     * @param yd Distorted normalized Y coordinate.
     * @param[out] xn Undistorted normalized X coordinate.
     * @param[out] yn Undistorted normalized Y coordinate.
     * @param max_iterations Maximum fixed-point iterations (default: 10).
     */
    void remove(double xd, double yd, double& xn, double& yn, int max_iterations = 10) const {
        if (is_identity()) {
            xn = xd;
            yn = yd;
            return;
        }
        // Fixed-point iteration initialized at distorted position
        xn = xd;
        yn = yd;
        for (int i = 0; i < max_iterations; ++i) {
            double r2 = xn * xn + yn * yn;
            double radial = 1.0 + k1 * r2 + k2 * r2 * r2;
            if (std::abs(radial) < 1e-12) break;
            xn = xd / radial;
            yn = yd / radial;
        }
    }
};

} // namespace star_tracker
