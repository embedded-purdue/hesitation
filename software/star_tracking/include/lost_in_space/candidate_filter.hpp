/**
 * @file candidate_filter.hpp
 * @brief Rapid rejection of incompatible database images via 6 geometric centroid invariants.
 *
 * @details
 * In Delabie et al. (2013, "SDT", Section II.E.1 "Discarding False Images"), directly evaluating
 * all N_database images against the distance transform map is unnecessary. By comparing 6 invariant
 * geometric features derived from bright-star triplets, more than 90% of database candidates can be
 * pruned instantly before performing any distance lookup.
 *
 * The 6 Invariant Geometric Features:
 * 1. Three Centroid Distances:
 *    | d_A^{cam} - d_A^{ref} | <= distance_tolerance_pixels
 *    | d_B^{cam} - d_B^{ref} | <= distance_tolerance_pixels
 *    | d_C^{cam} - d_C^{ref} | <= distance_tolerance_pixels
 *    where d_i is the Euclidean distance from the 3-star centroid to star i.
 * 2. Three Subtended Angles:
 *    | alpha_{min}^{cam} - alpha_{min}^{ref} | <= angle_tolerance_rad
 *    | alpha_{AC}^{cam} - alpha_{AC}^{ref} |   <= angle_tolerance_rad
 *    | alpha_{BC}^{cam} - alpha_{BC}^{ref} |   <= angle_tolerance_rad
 *
 * Conservative Tolerances:
 * As emphasized by Delabie et al., the tolerances are intentionally chosen to be generous
 * (e.g., 6.0 pixels and 0.04 rad / ~2.3 deg) to ensure that true database matches are never
 * accidentally eliminated, even under severe optical distortions (up to 300 arcseconds 1-sigma)
 * or large centroiding noise.
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." Journal of Guidance, Control, and Dynamics, 36(2),
 *   pp. 476–488. Section II.E.1 "Discarding False Images". DOI: 10.2514/1.56860.
 */

#pragma once

#include "database/reference_image.hpp"
#include <cmath>

namespace star_tracker {

/**
 * @struct CandidateFilterConfig
 * @brief Tolerance thresholds for 6-feature geometric candidate rejection.
 */
struct CandidateFilterConfig {
    double distance_tolerance_pixels{6.0}; ///< Maximum allowed deviation on centroid-to-star distance (pixels)
    double angle_tolerance_rad{0.04};       ///< Maximum allowed deviation on pairwise subtended angles (~2.3 degrees)
};

/**
 * @class CandidateFilter
 * @brief Evaluates geometric compatibility between camera and reference centroid features.
 */
class CandidateFilter {
public:
    /**
     * @brief Constructor with optional filter configuration.
     * @param config Feature matching tolerances.
     */
    explicit CandidateFilter(const CandidateFilterConfig& config = CandidateFilterConfig())
        : config_(config) {}

    /**
     * @brief Tests if camera and reference centroid features match within configured tolerances.
     * @details
     * Validates all 3 centroid distances and all 3 subtended angles against configured margins.
     * If any feature violates tolerance, returns false immediately.
     *
     * @param cam_features Triplet features extracted from camera image.
     * @param ref_features Precomputed triplet features from database reference image.
     * @return true if all 6 features match within tolerance, false otherwise.
     */
    bool matches(
        const CentroidFeatures& cam_features,
        const CentroidFeatures& ref_features
    ) const {
        if (!cam_features.valid || !ref_features.valid) {
            return false;
        }

        // Check 3 centroid-to-star radial distances
        for (int i = 0; i < 3; ++i) {
            double d_diff = std::abs(cam_features.distances[i] - ref_features.distances[i]);
            if (d_diff > config_.distance_tolerance_pixels) {
                return false;
            }

            // Check 3 pairwise subtended angles
            double a_diff = std::abs(cam_features.angles[i] - ref_features.angles[i]);
            if (a_diff > config_.angle_tolerance_rad) {
                return false;
            }
        }

        return true;
    }

    /// Read-only access to filter configuration.
    const CandidateFilterConfig& config() const { return config_; }

    /// Updates filter configuration.
    void set_config(const CandidateFilterConfig& config) { config_ = config; }

private:
    CandidateFilterConfig config_;
};

} // namespace star_tracker
