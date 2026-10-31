/**
 * @file test_lost_in_space.cpp
 * @brief Unit tests for autonomous Lost-in-Space identification under nominal and high-noise conditions.
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." Journal of Guidance, Control, and Dynamics, 36(2),
 *   pp. 476–488. Algorithm 1, Section III.C "False Stars". DOI: 10.2514/1.56860.
 */

#include <gtest/gtest.h>
#include "camera/camera_model.hpp"
#include "catalog/star_catalog.hpp"
#include "database/database_generator.hpp"
#include "database/reference_database.hpp"
#include "lost_in_space/distance_transform.hpp"
#include "lost_in_space/lost_in_space_matcher.hpp"
#include <cmath>
#include <random>

using namespace star_tracker;

class LostInSpaceTest : public ::testing::Test {
protected:
    void SetUp() override {
        double fov_rad = 20.0 * M_PI / 180.0;
        camera = CameraModel::from_fov(1024, 1024, fov_rad, fov_rad);

        // Load catalog or create synthetic catalog if file missing
        if (!catalog.load_from_csv("data/catalog/hipparcos.csv")) {
            // Add a realistic constellation of stars
            for (int i = 0; i < 200; ++i) {
                double ra = (i * 0.1);
                double dec = -0.5 + (i * 0.005);
                catalog.add_star(Star(i + 1, ra, dec, 3.0 + (i % 30) * 0.1));
            }
        }

        DatabaseGeneratorConfig cfg;
        cfg.num_images = 100;
        cfg.magnitude_threshold = 5.5;
        DatabaseGenerator gen(cfg);
        database = gen.generate(catalog, camera);
    }

    CameraModel camera;
    StarCatalog catalog;
    ReferenceDatabase database;
};

// Verifies clean lost-in-space identification and QUEST attitude determination without noise
TEST_F(LostInSpaceTest, CleanIdentification) {
    ASSERT_FALSE(database.empty());

    // Pick reference image 5 as ground truth attitude
    const auto& ground_truth_ref = database.images()[4];
    Quaternion true_att = ground_truth_ref.attitude;

    std::vector<ImageStar> visible = catalog.project_visible_stars(true_att, camera, 5.5);
    ASSERT_GE(visible.size(), 3u);

    DistanceTransform dt;
    dt.compute(visible, camera.width, camera.height);

    LostInSpaceMatcher matcher;
    AttitudeEstimate est = matcher.identify(visible, camera, database, dt);

    EXPECT_TRUE(est.valid);
    EXPECT_GT(est.confidence, 0.5);

    double err_deg = Quaternion::angular_distance(est.attitude, true_att) * 180.0 / M_PI;
    EXPECT_LT(err_deg, 1.0); // Within 1 degree of ground truth
}

// Verifies Delabie's false star immunity (Section III.C: retains >99% identification with false stars)
TEST_F(LostInSpaceTest, RobustnessToFalseStars) {
    ASSERT_FALSE(database.empty());

    const auto& ground_truth_ref = database.images()[10];
    Quaternion true_att = ground_truth_ref.attitude;

    std::vector<ImageStar> visible = catalog.project_visible_stars(true_att, camera, 5.5);
    ASSERT_GE(visible.size(), 4u);

    // Add 50 false stars scattered across the image (simulating radiation hits / space debris)
    std::mt19937 rng(1337);
    std::uniform_real_distribution<double> dist_pos(10.0, 1010.0);
    std::uniform_real_distribution<double> dist_mag(4.5, 6.0);

    for (int i = 0; i < 50; ++i) {
        visible.emplace_back(90000 + i, dist_pos(rng), dist_pos(rng), 100.0, dist_mag(rng));
    }

    DistanceTransform dt;
    dt.compute(visible, camera.width, camera.height);

    LostInSpaceMatcher matcher;
    AttitudeEstimate est = matcher.identify(visible, camera, database, dt);

    EXPECT_TRUE(est.valid);
    double err_deg = Quaternion::angular_distance(est.attitude, true_att) * 180.0 / M_PI;
    EXPECT_LT(err_deg, 1.0);
}
