/**
 * @file lost_in_space_matcher.cpp
 * @brief Implementation of Delabie's Shortest Distance Transform matching algorithm and QUEST solver.
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." Journal of Guidance, Control, and Dynamics, 36(2),
 *   pp. 476–488. Algorithm 1, Section II.D "Image Selection", Section II.E. DOI: 10.2514/1.56860.
 * - Shuster, M. D., and Oh, S. D. (1981). "Three-Axis Attitude Determination from Vector Observations."
 *   Journal of Guidance and Control, 4(1), pp. 70–77. DOI: 10.2514/3.19717.
 */

#include "aim/aim_estimator.hpp"
#include "lost_in_space/lost_in_space_matcher.hpp"
#include "math/wahba.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace star_tracker {

CandidateMatch LostInSpaceMatcher::identify_detailed(
    const std::vector<ImageStar>& camera_stars,
    const CameraModel& camera,
    const ReferenceDatabase& database,
    const DistanceTransform& dt
) {
    CandidateMatch best_match;
    best_match.valid = false;

    if (camera_stars.size() < 3 || database.empty()) {
        return best_match;
    }

    // Stage 1: Generate camera feature triplets from combinations of brightest stars
    // (Delabie et al., 2013, Section II.C.2, Table 1: combinations of 3 out of max_bright_stars)
    int max_s = std::min(config_.max_bright_stars, static_cast<int>(camera_stars.size()));
    std::vector<CentroidFeatures> cam_features_list;

    for (int i = 0; i < max_s; ++i) {
        for (int j = i + 1; j < max_s; ++j) {
            for (int k = j + 1; k < max_s; ++k) {
                std::vector<ImageStar> triplet = {camera_stars[i], camera_stars[j], camera_stars[k]};
                CentroidFeatures feat = CentroidFeatures::compute(triplet);
                if (feat.valid) {
                    cam_features_list.push_back(feat);
                }
            }
        }
    }

    if (cam_features_list.empty()) {
        return best_match;
    }

    double best_score = -1.0;

    // Stage 2: Evaluate against reference database (Algorithm 1, loop over database images)
    for (const auto& ref : database.images()) {
        if (ref.stars.empty()) {
            continue;
        }

        // Test against each precomputed reference feature combination
        for (const auto& ref_feat : ref.feature_combinations) {
            if (!ref_feat.valid) continue;

            for (const auto& cam_feat : cam_features_list) {
                // Section II.E.1: 6-feature geometric filter discards >90% of false database images
                if (!filter_.matches(cam_feat, ref_feat)) {
                    continue;
                }

                // Geometric filter passed: compute 2D translation and roll alignment
                Alignment align = CentroidMatcher::compute_alignment(cam_feat, ref_feat);
                if (!align.valid) continue;

                // Stage 3: Query 2D Shortest Distance Transform map for each reference star
                // (Delabie et al., 2013, Section II.D, Fig. 7)
                std::vector<double> distances;
                distances.reserve(ref.stars.size());
                int close_count = 0;
                double sum_close_dist = 0.0;
                std::vector<ImageStar> curr_matched_cam;
                std::vector<ImageStar> curr_matched_ref;

                for (const auto& rstar : ref.stars) {
                    double xc = 0.0, yc = 0.0;
                    align.transform_point(rstar.x, rstar.y, xc, yc);

                    int ix = static_cast<int>(std::round(xc));
                    int iy = static_cast<int>(std::round(yc));

                    double d = 0.0;
                    if (camera.is_in_bounds(ix, iy)) {
                        // O(1) distance query directly from precomputed lookup table!
                        uint32_t d2 = dt.at(ix, iy);
                        d = std::sqrt(static_cast<double>(d2));
                    } else {
                        // Penalty distance for projected stars falling outside camera sensor
                        d = 100.0;
                    }

                    distances.push_back(d);

                    // Section II.D.2: Close Stars Criterion (distance <= max_close_distance pixels)
                    if (d <= config_.max_close_distance) {
                        close_count++;
                        sum_close_dist += d;

                        // Identify nearest camera star spot corresponding to this reference star
                        int best_c = -1;
                        double best_cd2 = (config_.max_close_distance + 1.0) * (config_.max_close_distance + 1.0);
                        for (size_t c_idx = 0; c_idx < camera_stars.size(); ++c_idx) {
                            double cdx = camera_stars[c_idx].x - xc;
                            double cdy = camera_stars[c_idx].y - yc;
                            double cd2 = cdx * cdx + cdy * cdy;
                            if (cd2 < best_cd2) {
                                best_cd2 = cd2;
                                best_c = static_cast<int>(c_idx);
                            }
                        }
                        if (best_c >= 0) {
                            curr_matched_cam.push_back(camera_stars[best_c]);
                            curr_matched_ref.push_back(rstar);
                        }
                    }
                }

                std::sort(distances.begin(), distances.end());

                // Calculate ratio of close stars: N_close / N_ref
                double ratio = static_cast<double>(close_count) / static_cast<double>(ref.stars.size());

                // Delabie composite score: weighted by close star count, match percentage, and tightness
                double score = close_count * 100.0 + ratio * 10.0 - sum_close_dist;

                if (score > best_score) {
                    best_score = score;
                    best_match.reference = &ref;
                    best_match.cam_feature = cam_feat;
                    best_match.ref_feature = ref_feat;
                    best_match.alignment = align;
                    best_match.num_close_stars = close_count;
                    best_match.total_reference_stars = static_cast<int>(ref.stars.size());
                    best_match.close_star_ratio = ratio;
                    best_match.sorted_distances = distances;
                    best_match.matched_camera_stars = curr_matched_cam;
                    best_match.matched_reference_stars = curr_matched_ref;
                    best_match.confidence = ratio;
                }
            }
        }
    }

    // Stage 4: Validate acceptance criteria (Section II.D.1 & II.D.2)
    if (best_match.reference != nullptr) {
        if (best_match.num_close_stars >= config_.min_close_stars &&
            best_match.close_star_ratio >= config_.min_close_ratio) {
            best_match.valid = true;
        }
    }

    return best_match;
}

AttitudeEstimate LostInSpaceMatcher::identify(
    const std::vector<ImageStar>& camera_stars,
    const CameraModel& camera,
    const ReferenceDatabase& database,
    const DistanceTransform& dt
) {
    CandidateMatch match = identify_detailed(camera_stars, camera, database, dt);
    if (!match.valid || match.reference == nullptr) {
        return AttitudeEstimate(Quaternion::identity(), 0.0, false);
    }

    // Stage 5: Convert 2D focal-plane alignment into a differential rotation vector centered at optical axis
    double cx = camera.principal_x;
    double cy = camera.principal_y;

    double u_cam = match.cam_feature.centroid_x - cx;
    double v_cam = match.cam_feature.centroid_y - cy;
    double u_ref = match.ref_feature.centroid_x - cx;
    double v_ref = match.ref_feature.centroid_y - cy;

    double rot = match.alignment.rotation;
    double c = std::cos(rot);
    double s = std::sin(rot);

    double dx = u_cam - (u_ref * c - v_ref * s);
    double dy = v_cam - (u_ref * s + v_ref * c);

    double f = camera.focal_length;
    double theta_x = -dy / f;
    double theta_y =  dx / f;
    double theta_z =  rot;

    Vector3 rot_vec(theta_x, theta_y, theta_z);
    Quaternion q_align = Quaternion::from_rotation_vector(rot_vec);

    Quaternion q_est;

    // Stage 6: Full 3D attitude refinement using Shuster's QUEST algorithm if >= 3 matched stars
    if (match.matched_camera_stars.size() >= 3) {
        std::vector<Vector3> cam_rays;
        std::vector<Vector3> inertial_rays;
        std::vector<double> weights;

        // Inverse of reference attitude maps reference camera rays to celestial inertial frame
        Quaternion q_ref_inv = match.reference->attitude.inverse();

        for (size_t i = 0; i < match.matched_camera_stars.size(); ++i) {
            const auto& cs = match.matched_camera_stars[i];
            const auto& rs = match.matched_reference_stars[i];

            // 3D camera line-of-sight ray from pixel coordinates
            Vector3 w_cam = camera.pixel_to_vector(cs.x, cs.y);
            // 3D reference ray in reference frame, then rotated into inertial ECI frame
            Vector3 v_ref_cam = camera.pixel_to_vector(rs.x, rs.y);
            Vector3 v_inertial = q_ref_inv.rotate(v_ref_cam);

            cam_rays.push_back(w_cam);
            inertial_rays.push_back(v_inertial);
            weights.push_back(1.0);
        }

        // Solve Wahba's problem using QUEST (Shuster & Oh, 1981)
        q_est = solve_quest(cam_rays, inertial_rays, weights);
        if (q_est.norm_squared() < 0.5) {
            // Fallback to differential alignment if QUEST encountered near-singular geometry
            q_est = (q_align * match.reference->attitude).normalized();
        }
    } else {
        q_est = (q_align * match.reference->attitude).normalized();
    }

    return AttitudeEstimate(q_est, match.confidence, true);
}

} // namespace star_tracker
