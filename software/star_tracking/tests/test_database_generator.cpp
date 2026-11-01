/**
 * @file test_database_generator.cpp
 * @brief Unit tests for sphere covering algorithms and RA boundary calculations.
 *
 * @author Eric Yu
 * @date October 2026
 * @version 1.0.0
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." JGCD, 36(2), pp. 476–488.
 *   Section II.B, Eq. (3), citation [14]. DOI: 10.2514/1.56860.
 */

#include <gtest/gtest.h>
#include "database/database_generator.hpp"
#include <cmath>

using namespace star_tracker;

// Verifies Delabie et al. (2013) Eq. 3: alpha_b = gamma / (2 * cos(delta))
TEST(DatabaseGeneratorTest, RaBoundaryFormula) {
    double fov_rad = 20.0 * M_PI / 180.0; // 20 degrees gamma

    // At equator (delta = 0 rad): alpha_b = gamma / 2 = 10 deg
    double ra_b_eq = DatabaseGenerator::calculate_ra_boundary(fov_rad, 0.0);
    EXPECT_NEAR(ra_b_eq, fov_rad / 2.0, 1e-9);

    // At 60 degrees declination: cos(60 deg) = 0.5, alpha_b = gamma = 20 deg
    double dec_60 = 60.0 * M_PI / 180.0;
    double ra_b_60 = DatabaseGenerator::calculate_ra_boundary(fov_rad, dec_60);
    EXPECT_NEAR(ra_b_60, fov_rad, 1e-9);

    // Near pole: clamped to pi
    double dec_pole = 89.9 * M_PI / 180.0;
    double ra_b_pole = DatabaseGenerator::calculate_ra_boundary(fov_rad, dec_pole);
    EXPECT_NEAR(ra_b_pole, M_PI, 1e-9);
}

// Verifies Fibonacci spiral sphere covering
TEST(DatabaseGeneratorTest, FibonacciSpherePoints) {
    uint32_t N = 300;
    auto points = DatabaseGenerator::generate_sphere_points(N, SphereDistributionMethod::FIBONACCI_SPIRAL);
    EXPECT_EQ(points.size(), N);

    int pos_z = 0, neg_z = 0;
    for (const auto& pt : points) {
        EXPECT_NEAR(pt.norm(), 1.0, 1e-9);
        if (pt.z >= 0.0) pos_z++;
        else neg_z++;
    }
    // Balanced hemisphere distribution
    EXPECT_NEAR(pos_z, neg_z, 2);
}

// Verifies Leopardi's Recursive Zonal Equal Area (EQ) Sphere Partitioning (Delabie et al. 2013 [14])
TEST(DatabaseGeneratorTest, LeopardiSpherePoints) {
    uint32_t N = 300;
    auto points = DatabaseGenerator::generate_sphere_points(N, SphereDistributionMethod::LEOPARDI_EQUAL_AREA);
    EXPECT_EQ(points.size(), N);

    int pos_z = 0, neg_z = 0;
    for (const auto& pt : points) {
        EXPECT_NEAR(pt.norm(), 1.0, 1e-9);
        if (pt.z >= 0.0) pos_z++;
        else neg_z++;
    }
    // Balanced hemisphere distribution
    EXPECT_NEAR(pos_z, neg_z, 4);
}
