/**
 * @file attitude_manager.hpp
 * @brief Finite state machine orchestrating star tracker acquisition, tracking, and recovery.
 *
 * @details
 * This class coordinates the multi-mode star tracker operational lifecycle:
 *
 * State Machine Architecture:
 *
 *                     +-----------------------------------+
 *                     |                                   |
 *                     v                                   |
 *             [ ACQUISITION ] (Lost-in-Space)             |
 *                     |                                   | Tracking Lost /
 *                     | LIS Success + AIM Refinement      | Slew Detected
 *                     v                                   |
 *             [  TRACKING   ] (High-Rate AIM)             |
 *                     |                                   |
 *                     | Tracking Error Exceeds Tolerance  |
 *                     v                                   |
 *             [  RECOVERY   ] (Immediate LIS Reacq) ------+
 *
 * Modes and Algorithmic Responsibilities:
 * 1. ACQUISITION Mode:
 *    - Invoked upon system startup or cold boot with zero prior attitude knowledge.
 *    - Computes Maurer linear-time Shortest Distance Transform (SDT).
 *    - Evaluates 6 invariant centroid features across bright star triplets.
 *    - Solves Davenport/Wahba attitude via QUEST (Shuster & Oh, 1981).
 *    - Refines coarse solution with AIM and transitions to TRACKING.
 *
 * 2. TRACKING Mode:
 *    - High-frequency recursive focal-plane attitude determination (~2.4 microseconds).
 *    - Uses attitude solution from previous frame as coarse attitude q_dat.
 *    - Evaluates closed-form AIM optimization (Delabie et al., 2013) directly in 2D focal coordinates.
 *    - Eliminates expensive 3D ray conversion and matrix decompositions on every frame.
 *
 * 3. RECOVERY Mode:
 *    - Triggered immediately if AIM residuals exceed tolerance or too few stars are matched
 *      (e.g., due to thruster firings, fast slew maneuvers, or optical occultation).
 *    - Re-executes the Lost-in-Space pipeline for rapid autonomous reacquisition without
 *      interfering with attitude control loops.
 *
 * @author Eric Yu
 * @date October 2026
 * @version 1.0.0
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." JGCD, 36(2), pp. 476–488. DOI: 10.2514/1.56860.
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." JGCD, 36(6), pp. 1672–1680.
 *   DOI: 10.2514/1.61082.
 */

#pragma once

#include "aim/aim_estimator.hpp"
#include "camera/camera_model.hpp"
#include "catalog/star_catalog.hpp"
#include "database/reference_database.hpp"
#include "detection/image.hpp"
#include "detection/star_detector.hpp"
#include "lost_in_space/distance_transform.hpp"
#include "lost_in_space/lost_in_space_matcher.hpp"
#include "tracker/attitude_estimate.hpp"
#include <memory>

namespace star_tracker {

/**
 * @enum TrackerMode
 * @brief Discrete operational modes of the star tracker state machine.
 */
enum class TrackerMode {
    ACQUISITION, ///< Autonomous cold-boot star identification (Lost-in-Space SDT + QUEST)
    TRACKING,    ///< High-rate closed-form focal-plane attitude tracking (AIM)
    RECOVERY     ///< Immediate reacquisition following tracking loss or slew maneuver
};

/**
 * @brief Converts TrackerMode enum to string representation for logging and telemetry.
 * @param mode Operational tracker mode.
 * @return const char* String representation of the mode name.
 */
inline const char* to_string(TrackerMode mode) {
    switch (mode) {
        case TrackerMode::ACQUISITION: return "ACQUISITION";
        case TrackerMode::TRACKING:    return "TRACKING";
        case TrackerMode::RECOVERY:    return "RECOVERY";
        default: return "UNKNOWN";
    }
}

/**
 * @class AttitudeManager
 * @brief Core finite state machine managing star tracking modes, frame execution, and recovery.
 */
class AttitudeManager {
public:
    /**
     * @brief Constructs AttitudeManager with hardware models and reference data.
     * @param camera Camera optical geometry and distortion model.
     * @param database Precomputed reference star image database.
     * @param catalog Star catalog containing reference coordinates and magnitudes.
     */
    AttitudeManager(
        const CameraModel& camera,
        const ReferenceDatabase& database,
        const StarCatalog& catalog
    );

    /**
     * @brief Processes a raw incoming camera image frame.
     * @details
     * Detects stars via `StarDetector` and passes centroids to `process_stars`.
     * @param image Camera image buffer.
     * @return AttitudeEstimate containing attitude quaternion, validity, and confidence.
     */
    AttitudeEstimate process_frame(const Image& image);

    /**
     * @brief Processes a list of pre-extracted star centroids directly.
     * @details
     * Enables Hardware-In-The-Loop (HITL) bypass mode when centroids are computed by an
     * external hardware FPGA or optical simulation testbed.
     *
     * @param detected_stars List of 2D star centroids.
     * @return AttitudeEstimate attitude solution.
     */
    AttitudeEstimate process_stars(const std::vector<ImageStar>& detected_stars);

    /**
     * @brief Current operational mode of the state machine.
     * @return TrackerMode Active operational mode (ACQUISITION, TRACKING, or RECOVERY).
     */
    TrackerMode mode() const { return mode_; }

    /**
     * @brief Most recent attitude estimate.
     * @return const AttitudeEstimate& Reference to latest attitude telemetry.
     */
    const AttitudeEstimate& current_estimate() const { return current_estimate_; }

    /**
     * @brief Resets the state machine back to ACQUISITION mode (e.g. after mode change or reboot).
     */
    void reset() {
        mode_ = TrackerMode::ACQUISITION;
        current_estimate_ = AttitudeEstimate(Quaternion::identity(), 0.0, false);
        current_ref_image_ = nullptr;
    }

    /**
     * @brief Access to star detection submodule for parameter tuning.
     * @return StarDetector& Mutable reference to star detector.
     */
    StarDetector& detector() { return detector_; }

    /**
     * @brief Access to lost-in-space matcher for parameter tuning.
     * @return LostInSpaceMatcher& Mutable reference to LIS matcher.
     */
    LostInSpaceMatcher& lost_in_space() { return lost_in_space_; }

    /**
     * @brief Access to AIM estimator for parameter tuning.
     * @return AIMEstimator& Mutable reference to AIM estimator.
     */
    AIMEstimator& aim() { return aim_; }

    /**
     * @brief Access to camera model.
     * @return const CameraModel& Reference to intrinsic camera model.
     */
    const CameraModel& camera() const { return camera_; }

    /**
     * @brief Access to catalog.
     * @return const StarCatalog& Reference to star catalog.
     */
    const StarCatalog& catalog() const { return catalog_; }

    /**
     * @brief Access to reference database.
     * @return const ReferenceDatabase& Reference to precomputed reference database.
     */
    const ReferenceDatabase& database() const { return database_; }

private:
    /**
     * @brief Searches for the database reference image geometrically closest to an attitude.
     * @param att Query attitude quaternion.
     * @return const ReferenceImage* Pointer to closest reference image, or nullptr if database is empty.
     */
    const ReferenceImage* find_closest_reference(const Quaternion& att) const;

    CameraModel camera_;
    ReferenceDatabase database_;
    StarCatalog catalog_;

    TrackerMode mode_{TrackerMode::ACQUISITION};
    AttitudeEstimate current_estimate_;
    ReferenceImage tracking_ref_;
    const ReferenceImage* current_ref_image_{nullptr};

    StarDetector detector_;
    DistanceTransform dt_;
    LostInSpaceMatcher lost_in_space_;
    AIMEstimator aim_;
};

} // namespace star_tracker
