/**
 * @file database_generator.cpp
 * @brief Implementation of Fibonacci sphere reference image distribution and feature generation.
 *
 * @author Eric Yu
 * @date October 2026
 * @version 1.0.0
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." Journal of Guidance, Control, and Dynamics, 36(2),
 *   pp. 476–488. Section II.B & II.E.2. DOI: 10.2514/1.56860.
 */

#include "database/database_generator.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

namespace star_tracker {

/**
 * @brief Generates N unit vectors covering S^2 using the specified distribution method.
 * @param num_points Total number of points.
 * @param method Distribution algorithm (Fibonacci spiral or Leopardi EQ).
 * @return std::vector<Vector3> Vector of normalized 3D points on unit sphere.
 */
std::vector<Vector3> DatabaseGenerator::generate_sphere_points(
    uint32_t num_points,
    SphereDistributionMethod method
) {
    std::vector<Vector3> points;
    if (num_points == 0) return points;
    if (num_points == 1) {
        points.emplace_back(0.0, 0.0, 1.0);
        return points;
    }

    if (method == SphereDistributionMethod::LEOPARDI_EQUAL_AREA) {
        // Leopardi's Recursive Zonal Equal Area (EQ) Sphere Partitioning (Leopardi 2006, Delabie et al. 2013 [14])
        double area_per_region = 4.0 * M_PI / static_cast<double>(num_points);
        double s = std::sqrt(area_per_region);

        // North and south polar caps
        double theta_cap = 2.0 * std::asin(std::sqrt(1.0 / static_cast<double>(num_points)));
        double collar_span = M_PI - 2.0 * theta_cap;
        int n_collars = std::max(1, static_cast<int>(std::round(collar_span / s)));

        // North pole
        points.emplace_back(0.0, 0.0, 1.0);

        // Compute points per collar
        std::vector<int> m(n_collars);
        int total_assigned = 0;
        for (int i = 0; i < n_collars; ++i) {
            double ct0 = std::cos(theta_cap + i * collar_span / n_collars);
            double ct1 = std::cos(theta_cap + (i + 1) * collar_span / n_collars);
            double collar_area = 2.0 * M_PI * (ct0 - ct1);
            m[i] = static_cast<int>(std::round(collar_area / area_per_region));
            total_assigned += m[i];
        }

        // Adjust rounding discrepancies to ensure total points == num_points - 2
        int target_collar_points = static_cast<int>(num_points) - 2;
        int diff = target_collar_points - total_assigned;
        if (!m.empty()) {
            m[n_collars / 2] += diff;
            if (m[n_collars / 2] < 1) m[n_collars / 2] = 1;
        }

        for (int i = 0; i < n_collars; ++i) {
            double th_mid = theta_cap + (i + 0.5) * collar_span / n_collars;
            int num_pts = std::max(1, m[i]);
            double phase = (i % 2) * (M_PI / static_cast<double>(num_pts));
            for (int j = 0; j < num_pts; ++j) {
                double phi = 2.0 * M_PI * j / static_cast<double>(num_pts) + phase;
                double x = std::sin(th_mid) * std::cos(phi);
                double y = std::sin(th_mid) * std::sin(phi);
                double z = std::cos(th_mid);
                points.emplace_back(x, y, z);
            }
        }

        // South pole
        points.emplace_back(0.0, 0.0, -1.0);

        // If slight discrepancy remains, trim or duplicate
        while (points.size() > num_points) points.pop_back();
        while (points.size() < num_points) {
            double y = 1.0 - (2.0 * points.size()) / (num_points - 1);
            double r = std::sqrt(std::max(0.0, 1.0 - y * y));
            points.emplace_back(r, y, 0.0);
        }
        return points;
    }

    // Default: Fibonacci spiral covering
    const double phi_golden = M_PI * (3.0 - std::sqrt(5.0)); // ~2.39996 rad
    for (uint32_t i = 0; i < num_points; ++i) {
        double y = (num_points == 1) ? 0.0 : 1.0 - (2.0 * i) / (num_points - 1);
        double r = std::sqrt(std::max(0.0, 1.0 - y * y));
        double theta = phi_golden * i;
        double x = r * std::cos(theta);
        double z = r * std::sin(theta);
        points.emplace_back(x, y, z);
    }
    return points;
}

/**
 * @brief Generates a complete reference database from an astronomical star catalog.
 * @param catalog Astronomical star catalog.
 * @param camera Target camera intrinsic model and field of view.
 * @return ReferenceDatabase Generated reference database populated with reference images.
 */
ReferenceDatabase DatabaseGenerator::generate(
    const StarCatalog& catalog,
    const CameraModel& camera
) const {
    ReferenceDatabase db;
    uint32_t N = config_.num_images;
    if (N == 0) return db;

    // Filter catalog to magnitude threshold
    StarCatalog filtered = catalog.filter_by_magnitude(config_.magnitude_threshold);
    std::cout << "Generating reference database with " << N << " images from "
              << filtered.size() << " stars (mag <= " << config_.magnitude_threshold
              << ", method: " << (config_.distribution_method == SphereDistributionMethod::LEOPARDI_EQUAL_AREA ? "Leopardi EQ" : "Fibonacci spiral") << ")...\n";

    std::vector<Vector3> sphere_points = generate_sphere_points(N, config_.distribution_method);
    const Vector3 z_axis(0.0, 0.0, 1.0);

    for (uint32_t i = 0; i < sphere_points.size(); ++i) {
        Vector3 center_dir = sphere_points[i].normalized();

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
