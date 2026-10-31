/**
 * @file centroid_matcher.cpp
 * @brief Implementation of Delabie's 2D rigid transformation and triplet alignment.
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." Journal of Guidance, Control, and Dynamics, 36(2),
 *   pp. 476–488. Section II.C.2 "Centroid Method". DOI: 10.2514/1.56860.
 */

#include "lost_in_space/centroid_matcher.hpp"
#include "lost_in_space/candidate_filter.hpp"
#include <algorithm>
#include <cmath>

namespace star_tracker {

Alignment CentroidMatcher::compute_alignment(
    const CentroidFeatures& cam_features,
    const CentroidFeatures& ref_features
) {
    Alignment align_result;
    if (!cam_features.valid || !ref_features.valid) {
        align_result.valid = false;
        return align_result;
    }

    // Step 1: Compute roll angle difference between canonical triplet orientations
    // (Delabie et al., 2013, Section II.C.2, Fig. 6d-e)
    double rot = cam_features.orientation_angle - ref_features.orientation_angle;

    // Wrap angle to [-pi, pi]
    while (rot > M_PI) rot -= 2.0 * M_PI;
    while (rot < -M_PI) rot += 2.0 * M_PI;

    double c = std::cos(rot);
    double s = std::sin(rot);

    // Step 2: Compute translation (tx, ty) shifting reference centroid to camera centroid
    // (Delabie et al., 2013, Fig. 6c & 6f)
    double rx = ref_features.centroid_x;
    double ry = ref_features.centroid_y;

    double tx = cam_features.centroid_x - (c * rx - s * ry);
    double ty = cam_features.centroid_y - (s * rx + c * ry);

    align_result.translation_x = tx;
    align_result.translation_y = ty;
    align_result.rotation = rot;
    align_result.valid = true;

    return align_result;
}

Alignment CentroidMatcher::align(
    const std::vector<ImageStar>& camera_stars,
    const ReferenceImage& reference
) {
    if (camera_stars.size() < 3 || !reference.centroid_features.valid) {
        return Alignment();
    }

    CandidateFilter filter;
    int max_stars = std::min(5, static_cast<int>(camera_stars.size()));

    // Evaluate combinations of 3 stars among the top brightest camera stars
    // Combinations of 3 out of 5 = 10 combinations (Delabie et al. Table 1)
    for (int i = 0; i < max_stars; ++i) {
        for (int j = i + 1; j < max_stars; ++j) {
            for (int k = j + 1; k < max_stars; ++k) {
                std::vector<ImageStar> triplet = {camera_stars[i], camera_stars[j], camera_stars[k]};
                CentroidFeatures cam_feat = CentroidFeatures::compute(triplet);

                if (cam_feat.valid && filter.matches(cam_feat, reference.centroid_features)) {
                    return compute_alignment(cam_feat, reference.centroid_features);
                }
            }
        }
    }

    return Alignment();
}

} // namespace star_tracker
