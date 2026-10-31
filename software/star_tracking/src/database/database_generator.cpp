/**
 * @file database_generator.cpp
 * @brief Implementation of Fibonacci sphere reference image distribution and feature generation.
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." Journal of Guidance, Control, and Dynamics, 36(2),
 *   pp. 476–488. Section II.B & II.E.2. DOI: 10.2514/1.56860.
 */

#include "database/database_generator.hpp"
#include <cmath>
#include <iostream>

namespace star_tracker {

ReferenceDatabase DatabaseGenerator::generate(
    const StarCatalog& catalog,
    const CameraModel& camera
) const {
    ReferenceDatabase db;
    uint32_t N = config_.num_images;
    if (N == 0) return db;

    // Filter catalog to magnitude threshold
    // Delabie et al. (2013, Section II.B) note that magnitude 5.3-5.5 guarantees >= 3 stars per 20° FOV
    StarCatalog filtered = catalog.filter_by_magnitude(config_.magnitude_threshold);
    std::cout << "Generating reference database with " << N << " images from "
              << filtered.size() << " stars (mag <= " << config_.magnitude_threshold << ")...\n";

    // Golden angle in radians: phi_golden = pi * (3 - sqrt(5)) ≈ 2.39996 rad
    const double phi_golden = M_PI * (3.0 - std::sqrt(5.0));
    const Vector3 z_axis(0.0, 0.0, 1.0);

    for (uint32_t i = 0; i < N; ++i) {
        // Fibonacci spiral covering distribution on the unit sphere S^2
        double y = (N == 1) ? 0.0 : 1.0 - (2.0 * i) / (N - 1);
        double r = std::sqrt(std::max(0.0, 1.0 - y * y));
        double theta = phi_golden * i;
        double x = r * std::cos(theta);
        double z = r * std::sin(theta);

        Vector3 center_dir = Vector3(x, y, z).normalized();

        // Construct rotation quaternion q that aligns center_dir with camera optical axis +Z:
        // R(q) * center_dir = (0, 0, 1)^T
        Quaternion q;
        double dot_z = center_dir.dot(z_axis);
        if (dot_z > 0.9999999) {
            q = Quaternion::identity();
        } else if (dot_z < -0.9999999) {
            q = Quaternion::from_axis_angle(Vector3(1.0, 0.0, 0.0), M_PI);
        } else {
            Vector3 rot_axis = center_dir.cross(z_axis);
            double rot_ang = std::acos(std::clamp(dot_z, -1.0, 1.0));
            q = Quaternion::from_axis_angle(rot_axis, rot_ang);
        }

        // Project visible stars in camera field of view
        std::vector<ImageStar> visible = filtered.project_visible_stars(
            q, camera, config_.magnitude_threshold, config_.margin_pixels
        );

        ReferenceImage ref_img;
        ref_img.id = static_cast<uint64_t>(i + 1);
        ref_img.attitude = q;
        ref_img.stars = visible;

        // Precompute Delabie centroid features from combinations of brightest visible stars
        // Combinations of 3 out of 5 stars = 10 combinations (Delabie et al., 2013, Section II.E.2 & Table 1)
        int max_bright = std::min(5, static_cast<int>(visible.size()));
        for (int i1 = 0; i1 < max_bright; ++i1) {
            for (int i2 = i1 + 1; i2 < max_bright; ++i2) {
                for (int i3 = i2 + 1; i3 < max_bright; ++i3) {
                    std::vector<ImageStar> triplet = {visible[i1], visible[i2], visible[i3]};
                    CentroidFeatures feat = CentroidFeatures::compute(triplet);
                    if (feat.valid) {
                        ref_img.feature_combinations.push_back(feat);
                    }
                }
            }
        }

        if (!ref_img.feature_combinations.empty()) {
            ref_img.centroid_features = ref_img.feature_combinations[0];
        } else {
            ref_img.centroid_features.valid = false;
        }

        db.add_image(ref_img);
    }

    std::cout << "Reference database generation complete: " << db.size() << " images created.\n";
    return db;
}

} // namespace star_tracker
