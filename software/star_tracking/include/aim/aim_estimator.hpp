/**
 * @file aim_estimator.hpp
 * @brief Attitude Estimation using Image Matching (AIM) closed-form focal-plane attitude solver.
 *
 * @details
 * This class implements the breakthrough AIM algorithm developed by Delabie, De Schutter, and
 * Vandenbussche (2013).
 *
 * Core Concept:
 * Traditional attitude estimation algorithms (e.g. QUEST, ESOQ, FOAM, q-Method) require converting
 * all observed 2D star centroids into 3D unit vectors on every frame, which involves computationally
 * heavy division, square roots, and lens distortion corrections.
 * AIM fundamentally inverts this paradigm:
 * 1. Attitude determination is performed directly in 2D focal-plane coordinates.
 * 2. Only the reference catalog stars around coarse attitude q_dat are transformed, eliminating
 *    the coordinate conversion on detected camera stars.
 * 3. The 2D focal-plane rigid matching problem is solved analytically in closed-form without
 *    iterative matrix factorizations or eigenvalue decompositions.
 *
 * Mathematical Formulation:
 * Minimizes the weighted focal-plane Euclidean distance cost function (Eq. 9):
 *   Gamma(phi, tx, ty) = sum_{i=1}^{n_s} w_i * [ (x_i - x'_i)^2 + (y_i - y'_i)^2 ]
 * where transformed reference star coordinates are:
 *   x'_i =  x_hat_i * cos(phi) + y_hat_i * sin(phi) + tx
 *   y'_i = -x_hat_i * sin(phi) + y_hat_i * cos(phi) + ty
 *
 * Analytical Solution (Delabie et al., 2013, Eqs. 10–15):
 * Setting partial derivatives dGamma/dphi = 0, dGamma/dtx = 0, dGamma/dty = 0 yields:
 *   phi = atan2(S, C)  (Eq. 12)
 * with:
 *   C = sum w_i * ( Delta x_i * Delta x_hat_i + Delta y_i * Delta y_hat_i )
 *   S = sum w_i * ( Delta x_i * Delta y_hat_i - Delta y_i * Delta x_hat_i )
 * and center-of-mass translations:
 *   tx = x_bar - ( x_hat_bar * cos(phi) + y_hat_bar * sin(phi) )  (Eq. 13)
 *   ty = y_bar - (-x_hat_bar * sin(phi) + y_hat_bar * cos(phi) )  (Eq. 14)
 *
 * Attitude Quaternion Update:
 * Small-angle rotations relative to camera optical axis (focal length F):
 *   theta_x = -tv / F,  theta_y = tu / F,  theta_z = -phi  (Eqs. 16–18)
 *   q_diff = [ 1, 0.5*theta_x, 0.5*theta_y, 0.5*theta_z ]^T  (Eq. 20)
 * Total estimated attitude:
 *   q_e = q_diff ⊗ q_dat  (Eq. 21)
 *
 * Computational Performance:
 * Requires only 125 flops (4 stars) to 461 flops (25 stars), executing in ~2.4 microseconds
 * with sub-arcsecond accuracy.
 *
 * @reference
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." Journal of Guidance, Control,
 *   and Dynamics, 36(6), pp. 1672–1680. DOI: 10.2514/1.61082.
 */

#pragma once

#include "camera/camera_model.hpp"
#include "database/reference_image.hpp"
#include "math/quaternion.hpp"
#include <vector>

namespace star_tracker {

/**
 * @struct AIMConfig
 * @brief Configuration parameters and acceptance tolerances for AIM tracking.
 */
struct AIMConfig {
    double match_radius_pixels{10.0}; ///< Search gating window radius (pixels) around coarse predicted positions
    int min_matched_stars{3};          ///< Minimum matched stars required to compute valid attitude refinement
    double max_rms_error{2.5};         ///< Maximum permitted RMS focal-plane residual (pixels) for valid tracking
};

/**
 * @struct AIMResult
 * @brief Telemetry and state output from the closed-form AIM optimization.
 */
struct AIMResult {
    Quaternion corrected_attitude{Quaternion::identity()}; ///< Refined spacecraft attitude quaternion: q_e = q_diff ⊗ q_dat

    double image_rotation{0.0}; ///< Optimal in-plane roll rotation angle phi in radians (Eq. 12)
    double translation_x{0.0};  ///< Optimal horizontal focal-plane translation tx in pixels (Eq. 13)
    double translation_y{0.0};  ///< Optimal vertical focal-plane translation ty in pixels (Eq. 14)

    double cost{0.0};           ///< Final focal-plane residual cost value Gamma (Eq. 9)
    double rms_error{0.0};      ///< Root-Mean-Square focal-plane residual in pixels
    int num_matched_stars{0};   ///< Count of star pairs successfully matched and used in optimization

    bool valid{false};          ///< True if num_matched_stars >= min_matched and rms_error <= max_rms
};

/**
 * @class AIMEstimator
 * @brief Engine for optimal 2D focal-plane attitude determination via image matching.
 */
class AIMEstimator {
public:
    /**
     * @brief Constructor.
     * @param camera Camera optical geometry model.
     * @param config AIM tracking tolerances and gating thresholds.
     */
    explicit AIMEstimator(const CameraModel& camera, const AIMConfig& config = AIMConfig())
        : camera_(camera), config_(config) {}

    /**
     * @brief Refines attitude by matching detected camera stars against reference image stars.
     * @details
     * Applies gating radius around reference stars to associate star pairs, then executes
     * closed-form AIM optimization.
     *
     * @param observed_stars Detected camera star centroids.
     * @param reference Reference image containing catalog stars projected at coarse attitude.
     * @param coarse_attitude Prior coarse attitude quaternion q_dat.
     * @return AIMResult containing refined attitude, focal translations, and residuals.
     */
    AIMResult estimate(
        const std::vector<ImageStar>& observed_stars,
        const ReferenceImage& reference,
        const Quaternion& coarse_attitude
    );

    /**
     * @brief Closed-form optimal image matching for explicitly paired star sets.
     * @details
     * Evaluates Delabie et al. (2013) Eqs. (9–21) analytically:
     *   observed: (x_i, y_i), reference: (x_hat_i, y_hat_i), weights: w_i.
     *
     * @param observed_stars Star centroids in camera image.
     * @param reference_stars Corresponding star positions in reference frame.
     * @param weights Non-negative weighting factors for each star pair.
     * @param coarse_attitude Reference frame attitude quaternion q_dat.
     * @return AIMResult containing optimal attitude and transformation parameters.
     */
    AIMResult match_paired_stars(
        const std::vector<ImageStar>& observed_stars,
        const std::vector<ImageStar>& reference_stars,
        const std::vector<double>& weights,
        const Quaternion& coarse_attitude
    );

    /// Read-only access to AIM configuration.
    const AIMConfig& config() const { return config_; }

    /// Updates AIM configuration.
    void set_config(const AIMConfig& config) { config_ = config; }

    /// Read-only access to camera model.
    const CameraModel& camera() const { return camera_; }

    /// Updates camera model.
    void set_camera(const CameraModel& camera) { camera_ = camera; }

private:
    CameraModel camera_;
    AIMConfig config_;
};

} // namespace star_tracker
