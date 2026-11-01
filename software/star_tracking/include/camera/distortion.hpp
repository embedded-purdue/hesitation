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
 * @author Eric Yu
 * @date October 2026
 * @version 1.0.0
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

#include <array>
#include <cmath>

namespace star_tracker {

/**
 * @enum DistortionType
 * @brief Optical distortion model type.
 */
enum class DistortionType {
    RADIAL,              ///< Standard 3rd & 5th order radial distortion
    HERSCHEL_POLYNOMIAL  ///< Delabie et al. 2013 / Herschel STR-A 5th-order cross-polynomial (Table 4)
};

/**
 * @struct DistortionModel
 * @brief Optical lens distortion models supporting radial and 8-parameter cross-polynomials.
 */
struct DistortionModel {
    DistortionType type{DistortionType::RADIAL};

    // Radial parameters
    double k1{0.0}; ///< 3rd-order radial coefficient (applied to r^2)
    double k2{0.0}; ///< 5th-order radial coefficient (applied to r^4)

    // 8-parameter polynomial coefficients (Delabie et al., 2013, Eqs. 3-4 and 7-8, Table 4)
    std::array<double, 8> k_params{};     ///< k0..k7 for forward correction (x_c)
    std::array<double, 8> h_params{};     ///< h0..h7 for forward correction (y_c)
    std::array<double, 8> k_inv_params{}; ///< k0_inv..k7_inv for inverse distortion (x_d)
    std::array<double, 8> h_inv_params{}; ///< h0_inv..h7_inv for inverse distortion (y_d)

    /// Default constructor representing an ideal, distortion-free pinhole lens.
    constexpr DistortionModel() = default;

    /**
     * @brief Parameterized constructor for lens calibration parameters.
     * @param k1_ 3rd-order radial distortion coefficient.
     * @param k2_ 5th-order radial distortion coefficient.
     */
    constexpr DistortionModel(double k1_, double k2_)
        : type(DistortionType::RADIAL), k1(k1_), k2(k2_) {}

    /**
     * @brief Constructor for full 8-parameter polynomial distortion models.
     * @param k Coefficients k0..k7 for forward sensor x correction.
     * @param h Coefficients h0..h7 for forward sensor y correction.
     * @param k_inv Coefficients k0_inv..k7_inv for database x distortion.
     * @param h_inv Coefficients h0_inv..h7_inv for database y distortion.
     */
    DistortionModel(const std::array<double, 8>& k,
                    const std::array<double, 8>& h,
                    const std::array<double, 8>& k_inv,
                    const std::array<double, 8>& h_inv)
        : type(DistortionType::HERSCHEL_POLYNOMIAL), k1(0.0), k2(0.0),
          k_params(k), h_params(h), k_inv_params(k_inv), h_inv_params(h_inv) {}

    /**
     * @brief Creates a DistortionModel pre-loaded with the Herschel STR-A calibration parameters.
     * @reference Delabie et al. (2013), JGCD 36(6), Table 4.
     * @return DistortionModel Fully configured Herschel cross-polynomial model.
     */
    static DistortionModel herschel() {
        return DistortionModel(
            // k0..k7
            std::array<double, 8>{
                -1.27866006e-5, 1.0, 8.57471666e-5, 2.19016482e-4,
                -5.20164707e-7, -6.53773139e-5, -5.29575523e-5, -2.24629447e-5
            },
            // h0..h7
            std::array<double, 8>{
                2.0675277e-5, 1.0, -1.60933572e-5, 2.14748994e-4,
                -2.26158663e-7, -4.54670784e-5, -4.89014428e-5, -2.27073830e-5
            },
            // k0_inv..k7_inv
            std::array<double, 8>{
                1.3025586e-4, 0.99999929, -8.5120363e-5, 2.1871748e-4,
                6.3269083e-7, 6.4147788e-5, 5.2131004e-5, 2.2034513e-5
            },
            // h0_inv..h7_inv
            std::array<double, 8>{
                -1.875591e-5, 0.99999960, 1.6173263e-5, 2.1458371e-4,
                3.4805732e-7, 4.4630672e-5, 4.8010159e-5, 2.2337249e-5
            }
        );
    }

    /**
     * @brief Checks if the lens distortion is numerically zero (ideal pinhole).
     * @return true if distortion coefficients are negligibly small, false otherwise.
     */
    bool is_identity() const {
        if (type == DistortionType::RADIAL) {
            return std::abs(k1) < 1e-12 && std::abs(k2) < 1e-12;
        }
        return false;
    }

    /**
     * @brief Applies forward lens distortion to undistorted normalized coordinates.
     * @details
     * For radial: maps (xn, yn) to (xd, yd) = (xn * (1 + k1*r^2 + k2*r^4), yn * (1 + k1*r^2 + k2*r^4)).
     * For Herschel: maps ideal database coordinates to distorted coordinates via Eqs. (7-8).
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

        if (type == DistortionType::HERSCHEL_POLYNOMIAL) {
            double r2 = xn * xn + yn * yn;
            double r4 = r2 * r2;
            xd = -k_inv_params[0] + k_inv_params[1] * xn + k_inv_params[2] * yn
                 + k_inv_params[3] * xn * r2 + k_inv_params[4] * xn * r4
                 - k_inv_params[5] * xn * xn - k_inv_params[6] * xn * yn - k_inv_params[7] * yn * yn;
            yd = -h_inv_params[0] + h_inv_params[1] * yn + h_inv_params[2] * xn
                 + h_inv_params[3] * yn * r2 + h_inv_params[4] * yn * r4
                 - h_inv_params[5] * yn * yn - h_inv_params[6] * yn * xn - h_inv_params[7] * xn * xn;
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
     * For radial: computes undistorted (xn, yn) via fixed-point iteration.
     * For Herschel: evaluates closed-form correction polynomials from Eqs. (3-4).
     * @param xd Distorted normalized X coordinate.
     * @param yd Distorted normalized Y coordinate.
     * @param[out] xn Undistorted normalized X coordinate.
     * @param[out] yn Undistorted normalized Y coordinate.
     * @param max_iterations Maximum fixed-point iteration limit for radial inversion (default: 10).
     */
    void remove(double xd, double yd, double& xn, double& yn, int max_iterations = 10) const {
        if (is_identity()) {
            xn = xd;
            yn = yd;
            return;
        }

        if (type == DistortionType::HERSCHEL_POLYNOMIAL) {
            double r2 = xd * xd + yd * yd;
            double r4 = r2 * r2;
            xn = -k_params[0] + k_params[1] * xd + k_params[2] * yd
                 + k_params[3] * xd * r2 + k_params[4] * xd * r4
                 - k_params[5] * xd * xd - k_params[6] * xd * yd - k_params[7] * yd * yd;
            yn = -h_params[0] + h_params[1] * yd + h_params[2] * xd
                 + h_params[3] * yd * r2 + h_params[4] * yd * r4
                 - h_params[5] * yd * yd - h_params[6] * yd * xd - h_params[7] * xd * xd;
            return;
        }

        // Fixed-point iteration for radial model
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
