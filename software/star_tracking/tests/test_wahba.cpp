/**
 * @file test_wahba.cpp
 * @brief Unit tests comparing QUEST, Davenport's q-method, and SVD Wahba solvers.
 *
 * @author Eric Yu
 * @date October 2026
 * @version 1.0.0
 *
 * @reference
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." JGCD, 36(6), pp. 1672–1680.
 *   Sec. I & III. DOI: 10.2514/1.61082.
 */

#include <gtest/gtest.h>
#include "math/wahba.hpp"
#include <cmath>

using namespace star_tracker;

class WahbaTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Known ground truth rotation
        q_true = Quaternion::from_euler(0.15, -0.25, 0.35);

        // 5 widely spaced reference vectors
        ref_vectors = {
            Vector3(0.0, 0.0, 1.0).normalized(),
            Vector3(0.2, 0.1, 0.97).normalized(),
            Vector3(-0.15, 0.2, 0.96).normalized(),
            Vector3(0.1, -0.22, 0.97).normalized(),
            Vector3(-0.18, -0.15, 0.97).normalized()
        };

        for (const auto& r : ref_vectors) {
            cam_rays.push_back(q_true.rotate(r));
        }
    }

    Quaternion q_true;
    std::vector<Vector3> ref_vectors;
    std::vector<Vector3> cam_rays;
};

// Verifies identity behavior for all solvers
TEST_F(WahbaTest, IdentityRotation) {
    std::vector<Vector3> rays = ref_vectors;
    Quaternion q_quest = solve_quest(rays, ref_vectors);
    Quaternion q_dav = solve_q_davenport(rays, ref_vectors);
    Quaternion q_svd = solve_svd(rays, ref_vectors);

    EXPECT_NEAR(Quaternion::angular_distance(Quaternion::identity(), q_quest), 0.0, 1e-10);
    EXPECT_NEAR(Quaternion::angular_distance(Quaternion::identity(), q_dav), 0.0, 1e-10);
    EXPECT_NEAR(Quaternion::angular_distance(Quaternion::identity(), q_svd), 0.0, 1e-10);
}

// Verifies that QUEST, Davenport's q-method, and SVD all recover true attitude
TEST_F(WahbaTest, KnownRotationRecovery) {
    Quaternion q_quest = solve_quest(cam_rays, ref_vectors);
    Quaternion q_dav = solve_q_davenport(cam_rays, ref_vectors);
    Quaternion q_svd = solve_svd(cam_rays, ref_vectors);

    double err_quest = Quaternion::angular_distance(q_true, q_quest);
    double err_dav = Quaternion::angular_distance(q_true, q_dav);
    double err_svd = Quaternion::angular_distance(q_true, q_svd);

    EXPECT_NEAR(err_quest, 0.0, 1e-7);
    EXPECT_NEAR(err_dav, 0.0, 1e-7);
    EXPECT_NEAR(err_svd, 0.0, 1e-7);

    // Mutual agreement between QUEST and Davenport/SVD
    EXPECT_NEAR(Quaternion::angular_distance(q_quest, q_dav), 0.0, 1e-7);
    EXPECT_NEAR(Quaternion::angular_distance(q_quest, q_svd), 0.0, 1e-7);
}

// Verifies weighted vector observations
TEST_F(WahbaTest, WeightedObservations) {
    std::vector<double> weights = {3.0, 1.5, 0.5, 2.0, 1.0};
    Quaternion q_quest = solve_quest(cam_rays, ref_vectors, weights);
    Quaternion q_dav = solve_q_davenport(cam_rays, ref_vectors, weights);
    Quaternion q_svd = solve_svd(cam_rays, ref_vectors, weights);

    EXPECT_NEAR(Quaternion::angular_distance(q_true, q_quest), 0.0, 1e-8);
    EXPECT_NEAR(Quaternion::angular_distance(q_true, q_dav), 0.0, 1e-8);
    EXPECT_NEAR(Quaternion::angular_distance(q_true, q_svd), 0.0, 1e-8);
}

// Verifies handling of noisy vector measurements
TEST_F(WahbaTest, NoisyObservations) {
    std::vector<Vector3> noisy_rays = cam_rays;
    // Add 1 arcsecond noise (~5 microradians)
    noisy_rays[0] = (noisy_rays[0] + Vector3(5e-6, -3e-6, 0.0)).normalized();
    noisy_rays[1] = (noisy_rays[1] + Vector3(-4e-6, 5e-6, 0.0)).normalized();

    Quaternion q_quest = solve_quest(noisy_rays, ref_vectors);
    Quaternion q_dav = solve_q_davenport(noisy_rays, ref_vectors);
    Quaternion q_svd = solve_svd(noisy_rays, ref_vectors);

    // All should be within arcsecond level of true attitude
    EXPECT_LT(Quaternion::angular_distance(q_true, q_quest), 1e-4);
    EXPECT_LT(Quaternion::angular_distance(q_true, q_dav), 1e-4);
    EXPECT_LT(Quaternion::angular_distance(q_true, q_svd), 1e-4);

    // Solvers should agree closely even with noise
    EXPECT_NEAR(Quaternion::angular_distance(q_quest, q_dav), 0.0, 1e-6);
    EXPECT_NEAR(Quaternion::angular_distance(q_quest, q_svd), 0.0, 1e-6);
}
