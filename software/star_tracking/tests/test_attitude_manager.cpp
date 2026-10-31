/**
 * @file test_attitude_manager.cpp
 * @brief Unit tests for the AttitudeManager finite state machine transitions (ACQUISITION -> TRACKING -> RECOVERY).
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." Journal of Guidance, Control, and Dynamics, 36(2),
 *   pp. 476–488. DOI: 10.2514/1.56860.
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." Journal of Guidance, Control,
 *   and Dynamics, 36(6), pp. 1672–1680. DOI: 10.2514/1.61082.
 */

#include <gtest/gtest.h>
#include "camera/camera_model.hpp"
#include "catalog/star_catalog.hpp"
#include "database/database_generator.hpp"
#include "database/reference_database.hpp"
#include "tracker/attitude_manager.hpp"
#include <cmath>

using namespace star_tracker;

class AttitudeManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        double fov_rad = 20.0 * M_PI / 180.0;
        camera = CameraModel::from_fov(1024, 1024, fov_rad, fov_rad);

        if (!catalog.load_from_csv("data/catalog/hipparcos.csv")) {
            for (int i = 0; i < 200; ++i) {
                double ra = (i * 0.1);
                double dec = -0.5 + (i * 0.005);
                catalog.add_star(Star(i + 1, ra, dec, 3.0 + (i % 30) * 0.1));
            }
        }

        DatabaseGeneratorConfig cfg;
        cfg.num_images = 80;
        cfg.magnitude_threshold = 5.5;
        DatabaseGenerator gen(cfg);
        database = gen.generate(catalog, camera);
    }

    CameraModel camera;
    StarCatalog catalog;
    ReferenceDatabase database;
};

// Verifies finite state machine transitions across ACQUISITION, TRACKING, and RECOVERY modes
TEST_F(AttitudeManagerTest, OperatingSequence) {
    AttitudeManager manager(camera, database, catalog);
    EXPECT_EQ(manager.mode(), TrackerMode::ACQUISITION);

    // Pick reference image 3
    const auto& ref_target = database.images()[2];
    Quaternion true_att = ref_target.attitude;

    std::vector<ImageStar> stars_frame1 = catalog.project_visible_stars(true_att, camera, 5.5);
    ASSERT_GE(stars_frame1.size(), 3u);

    // Frame 1: ACQUISITION (Lost-in-Space) -> Transitions to TRACKING
    AttitudeEstimate est1 = manager.process_stars(stars_frame1);
    EXPECT_TRUE(est1.valid);
    EXPECT_EQ(manager.mode(), TrackerMode::TRACKING);

    // Frame 2: Small movement in TRACKING (AIM closed-form focal plane refinement)
    double arcsec = M_PI / (180.0 * 3600.0);
    Quaternion true_att2 = Quaternion::from_euler(30.0 * arcsec, 20.0 * arcsec, 40.0 * arcsec) * true_att;
    std::vector<ImageStar> stars_frame2 = catalog.project_visible_stars(true_att2, camera, 5.5);

    AttitudeEstimate est2 = manager.process_stars(stars_frame2);
    EXPECT_TRUE(est2.valid);
    EXPECT_EQ(manager.mode(), TrackerMode::TRACKING);

    // Frame 3: Instantaneous slew to a distant celestial region -> Triggers RECOVERY and immediate reacquisition
    const auto& ref_target_far = database.images()[25];
    Quaternion true_att3 = ref_target_far.attitude;
    std::vector<ImageStar> stars_frame3 = catalog.project_visible_stars(true_att3, camera, 5.5);

    AttitudeEstimate est3 = manager.process_stars(stars_frame3);
    EXPECT_TRUE(est3.valid);
    EXPECT_EQ(manager.mode(), TrackerMode::TRACKING);

    double err = Quaternion::angular_distance(est3.attitude, true_att3) * 180.0 / M_PI;
    EXPECT_LT(err, 1.0);
}
