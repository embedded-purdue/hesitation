/**
 * @file attitude_manager.cpp
 * @brief Implementation of AttitudeManager state machine transitions and AIM tracking loops.
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." Journal of Guidance, Control, and Dynamics, 36(2),
 *   pp. 476–488. DOI: 10.2514/1.56860.
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." Journal of Guidance, Control,
 *   and Dynamics, 36(6), pp. 1672–1680. DOI: 10.2514/1.61082.
 */

#include "tracker/attitude_manager.hpp"
#include <iostream>

namespace star_tracker {

AttitudeManager::AttitudeManager(
    const CameraModel& camera,
    const ReferenceDatabase& database,
    const StarCatalog& catalog
) : camera_(camera),
    database_(database),
    catalog_(catalog),
    aim_(camera) {}

const ReferenceImage* AttitudeManager::find_closest_reference(const Quaternion& att) const {
    const ReferenceImage* best = nullptr;
    double min_dist = 1e9;

    for (const auto& ref : database_.images()) {
        double d = Quaternion::angular_distance(att, ref.attitude);
        if (d < min_dist) {
            min_dist = d;
            best = &ref;
        }
    }
    return best;
}

AttitudeEstimate AttitudeManager::process_stars(const std::vector<ImageStar>& detected_stars) {
    // -------------------------------------------------------------------------
    // Mode 1: ACQUISITION / RECOVERY (Lost-in-Space)
    // -------------------------------------------------------------------------
    if (mode_ == TrackerMode::ACQUISITION || mode_ == TrackerMode::RECOVERY) {
        // Step 1: Compute Maurer exact linear-time Shortest Distance Transform (SDT)
        // (Delabie et al., 2013, "SDT", Section II.A)
        dt_.compute(detected_stars, camera_.width, camera_.height);

        // Step 2: Autonomous star identification & QUEST attitude solution
        AttitudeEstimate est = lost_in_space_.identify(detected_stars, camera_, database_, dt_);
        if (est.valid) {
            // Step 3: Refine the coarse Lost-in-Space estimate using AIM optimal image matching
            // (Delabie et al., 2013, "AIM", Section II.C & II.D)
            tracking_ref_.attitude = est.attitude;
            tracking_ref_.stars = catalog_.project_visible_stars(est.attitude, camera_, 5.5);

            AIMResult refine_res = aim_.estimate(detected_stars, tracking_ref_, est.attitude);
            if (refine_res.valid) {
                est.attitude = refine_res.corrected_attitude;
                est.confidence = 1.0;
                est.rms_error_pixels = refine_res.rms_error;
                est.matched_stars = refine_res.num_matched_stars;
            }

            current_estimate_ = est;
            mode_ = TrackerMode::TRACKING;
            current_ref_image_ = find_closest_reference(est.attitude);

            // Re-project tracking reference stars around refined attitude
            tracking_ref_.attitude = est.attitude;
            tracking_ref_.stars = catalog_.project_visible_stars(est.attitude, camera_, 5.5);
        } else {
            current_estimate_.valid = false;
        }
        return current_estimate_;
    }

    // -------------------------------------------------------------------------
    // Mode 2: TRACKING (High-Rate Closed-Form AIM Tracking)
    // -------------------------------------------------------------------------
    if (mode_ == TrackerMode::TRACKING) {
        // Update projected reference stars if spacecraft moved significantly from previous frame
        if (tracking_ref_.stars.empty() ||
            Quaternion::angular_distance(current_estimate_.attitude, tracking_ref_.attitude) > 0.05) {
            tracking_ref_.attitude = current_estimate_.attitude;
            tracking_ref_.stars = catalog_.project_visible_stars(current_estimate_.attitude, camera_, 5.5);
        }

        if (tracking_ref_.stars.empty()) {
            mode_ = TrackerMode::RECOVERY;
            current_estimate_.valid = false;
            return current_estimate_;
        }

        // Execute closed-form AIM optimization: computes optimal translation and rotation
        // in focal-plane coordinates (Delabie et al., 2013, "AIM", Section II.D, Eqs. 9–21)
        AIMResult aim_res = aim_.estimate(detected_stars, tracking_ref_, current_estimate_.attitude);

        if (aim_res.valid) {
            current_estimate_.attitude = aim_res.corrected_attitude;
            current_estimate_.valid = true;
            current_estimate_.rms_error_pixels = aim_res.rms_error;
            current_estimate_.matched_stars = aim_res.num_matched_stars;
            double norm_err = aim_res.rms_error / aim_.config().max_rms_error;
            current_estimate_.confidence = std::max(0.0, 1.0 - norm_err);
        } else {
            // Tracking lost -> Transition to RECOVERY mode
            mode_ = TrackerMode::RECOVERY;
            current_estimate_.valid = false;

            // Attempt immediate recovery within the same cycle via lost-in-space
            dt_.compute(detected_stars, camera_.width, camera_.height);
            AttitudeEstimate rec_est = lost_in_space_.identify(detected_stars, camera_, database_, dt_);
            if (rec_est.valid) {
                tracking_ref_.attitude = rec_est.attitude;
                tracking_ref_.stars = catalog_.project_visible_stars(rec_est.attitude, camera_, 5.5);

                AIMResult refine_res = aim_.estimate(detected_stars, tracking_ref_, rec_est.attitude);
                if (refine_res.valid) {
                    rec_est.attitude = refine_res.corrected_attitude;
                    rec_est.confidence = 1.0;
                    rec_est.rms_error_pixels = refine_res.rms_error;
                    rec_est.matched_stars = refine_res.num_matched_stars;
                }

                current_estimate_ = rec_est;
                mode_ = TrackerMode::TRACKING;
                current_ref_image_ = find_closest_reference(rec_est.attitude);
                tracking_ref_.attitude = rec_est.attitude;
                tracking_ref_.stars = catalog_.project_visible_stars(rec_est.attitude, camera_, 5.5);
            }
        }
    }

    return current_estimate_;
}

AttitudeEstimate AttitudeManager::process_frame(const Image& image) {
    std::vector<ImageStar> stars = detector_.detect_stars(image, camera_);
    return process_stars(stars);
}

} // namespace star_tracker
