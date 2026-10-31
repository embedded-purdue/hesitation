/**
 * @file lost_in_space_matcher.hpp
 * @brief Autonomous star identification and initial attitude determination via Shortest Distance Transform.
 *
 * @details
 * This class implements the complete Lost-in-Space (LIS) identification algorithm described in
 * Delabie, Durt, and Vandersteen (2013, "SDT", Algorithm 1, p. 480).
 *
 * Operational Sequence (Algorithm 1):
 * 1. Shortest Distance Transform:
 *    Generates exact 2D squared Euclidean distance lookup table D(x, y) from detected star centroids
 *    using Maurer's linear-time algorithm.
 * 2. Triplet Feature Generation:
 *    Extracts invariant geometric centroid features for combinations of the brightest camera stars.
 * 3. Fast Candidate Screening:
 *    Iterates over the reference database, applying `CandidateFilter` (6 invariant geometric features)
 *    to discard >90% of non-matching database images.
 * 4. 2D Alignment & Distance Map Query:
 *    For candidates passing the geometric filter, transforms reference stars into camera focal plane
 *    via 2D translation (t_x, t_y) and roll rotation phi.
 *    For each reference star, retrieves distance D(x, y) from the precomputed lookup table in O(1) time.
 * 5. Dual Verification Criteria (Delabie et al., Section II.D):
 *    - Close Stars Criterion: Counts database stars within max_close_distance (e.g., 2.0 pixels)
 *      of a camera star.
 *    - Close Star Ratio: Requires at least min_close_ratio (e.g., >= 50%) of reference stars to match.
 *    - Distance Criterion: Computes total distance sum of N_tot closest stars.
 * 6. Attitude Determination (QUEST):
 *    Transforms verified 2D star pairs into 3D camera and celestial inertial rays, solving
 *    Wahba's problem in closed form via Shuster's QUEST algorithm.
 *
 * Performance & Robustness:
 * Solves the lost-in-space problem in ~3.9 ms on a standard processor, maintaining >99%
 * identification accuracy under 400 false stars, 300 arcsec positional noise, or missing bright stars.
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." Journal of Guidance, Control, and Dynamics, 36(2),
 *   pp. 476–488. DOI: 10.2514/1.56860.
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." JGCD, 36(6), pp. 1672–1680.
 *   DOI: 10.2514/1.61082.
 */

#pragma once

#include "camera/camera_model.hpp"
#include "database/reference_database.hpp"
#include "lost_in_space/candidate_filter.hpp"
#include "lost_in_space/centroid_matcher.hpp"
#include "lost_in_space/distance_transform.hpp"
#include "tracker/attitude_estimate.hpp"
#include <vector>

namespace star_tracker {

/**
 * @struct LostInSpaceConfig
 * @brief Operational thresholds for the Delabie Shortest Distance Transform matcher.
 */
struct LostInSpaceConfig {
    CandidateFilterConfig filter_config; ///< 6-feature geometric pre-filter tolerances
    int max_bright_stars{5};          ///< Number of brightest camera stars to form triplet combinations (e.g. 5 stars -> 10 combinations)
    double max_close_distance{2.0};   ///< Maximum focal-plane distance in pixels to count as a close star match (Delabie et al. Section II.D.2)
    int min_close_stars{3};           ///< Minimum absolute count of matching stars required for positive identification
    double min_close_ratio{0.50};     ///< Minimum ratio of matched reference stars: N_close / N_tot >= 0.50
};

/**
 * @struct CandidateMatch
 * @brief Detailed diagnostic output from evaluating a candidate reference image against camera stars.
 */
struct CandidateMatch {
    const ReferenceImage* reference{nullptr}; ///< Pointer to best-matching database reference image
    CentroidFeatures cam_feature;            ///< Matched camera triplet feature
    CentroidFeatures ref_feature;            ///< Matched reference triplet feature
    Alignment alignment;                     ///< 2D rigid transformation (tx, ty, phi)

    int num_close_stars{0};                  ///< Number of stars within max_close_distance
    int total_reference_stars{0};            ///< Total reference stars in field of view
    double close_star_ratio{0.0};            ///< Fraction of reference stars matched: num_close / total_ref
    std::vector<double> sorted_distances;    ///< Sorted focal-plane Euclidean distances (pixels)
    std::vector<ImageStar> matched_camera_stars;    ///< Identified camera star spots
    std::vector<ImageStar> matched_reference_stars; ///< Corresponding reference catalog stars

    double confidence{0.0};                  ///< Normalized match confidence [0.0, 1.0]
    bool valid{false};                       ///< True if all acceptance criteria are satisfied
};

/**
 * @class LostInSpaceMatcher
 * @brief Autonomous Lost-in-Space star identification engine.
 */
class LostInSpaceMatcher {
public:
    /**
     * @brief Constructor with optional algorithm configuration.
     * @param config Operational parameters.
     */
    explicit LostInSpaceMatcher(const LostInSpaceConfig& config = LostInSpaceConfig())
        : config_(config), filter_(config.filter_config) {}

    /**
     * @brief Executes autonomous Lost-in-Space identification to determine spacecraft attitude.
     * @details
     * Implements Delabie et al. (2013) Algorithm 1, querying the distance transform map and
     * solving attitude via QUEST.
     *
     * @param camera_stars Detected camera star centroids.
     * @param camera Camera geometric and optical intrinsic model.
     * @param database Precomputed reference star database.
     * @param dt Precomputed exact Euclidean distance transform map of the camera frame.
     * @return AttitudeEstimate containing estimated quaternion, confidence metric, and validity flag.
     */
    AttitudeEstimate identify(
        const std::vector<ImageStar>& camera_stars,
        const CameraModel& camera,
        const ReferenceDatabase& database,
        const DistanceTransform& dt
    );

    /**
     * @brief Executes identification returning rich telemetry and diagnostic match structures.
     * @param camera_stars Detected camera star centroids.
     * @param camera Camera model.
     * @param database Reference database.
     * @param dt Distance transform lookup map.
     * @return CandidateMatch containing full diagnostic telemetry.
     */
    CandidateMatch identify_detailed(
        const std::vector<ImageStar>& camera_stars,
        const CameraModel& camera,
        const ReferenceDatabase& database,
        const DistanceTransform& dt
    );

    /// Read-only access to matcher configuration.
    const LostInSpaceConfig& config() const { return config_; }

    /// Updates matcher configuration.
    void set_config(const LostInSpaceConfig& config) {
        config_ = config;
        filter_.set_config(config.filter_config);
    }

private:
    LostInSpaceConfig config_;
    CandidateFilter filter_;
};

} // namespace star_tracker
