/**
 * @file test_distance_transform.cpp
 * @brief Unit tests validating Maurer's linear-time exact Euclidean distance transform against brute-force.
 *
 * @reference
 * - Maurer, C. R., Qi, R., and Raghavan, V. (2003). "A Linear Time Algorithm for Computing
 *   Exact Euclidean Distance Transforms of Binary Images in Arbitrary Dimensions." IEEE TPAMI,
 *   25(2), pp. 265–270. DOI: 10.1109/TPAMI.2003.1177156.
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." JGCD, 36(2), pp. 476–488. Section II.A
 *   "Distance Transform", Eqs. (1–2). DOI: 10.2514/1.56860.
 */

#include <gtest/gtest.h>
#include "lost_in_space/distance_transform.hpp"

using namespace star_tracker;

// Verifies sentinel behavior when computing distance map with zero star spots
TEST(DistanceTransformTest, EmptyImage) {
    DistanceTransform dt;
    dt.compute({}, 64, 64);
    EXPECT_GE(dt.at(10, 10), DistanceTransform::INF_DIST);
}

// Verifies exact squared Euclidean distance values around a single isolated star centroid
TEST(DistanceTransformTest, SingleStarExactDistance) {
    int w = 50, h = 50;
    std::vector<ImageStar> stars = {{1, 20.0, 30.0, 100.0, 3.0}};

    DistanceTransform dt;
    dt.compute(stars, w, h);

    // At the star location, squared distance must be identically 0
    EXPECT_EQ(dt.at(20, 30), 0u);

    // At (23, 34), dx=3, dy=4 -> dx^2 + dy^2 = 9 + 16 = 25
    EXPECT_EQ(dt.at(23, 34), 25u);

    // At (10, 30), dx=-10, dy=0 -> dx^2 + dy^2 = 100
    EXPECT_EQ(dt.at(10, 30), 100u);
}

// Verifies boundary stars at image extreme corners
TEST(DistanceTransformTest, BoundaryStars) {
    int w = 100, h = 100;
    std::vector<ImageStar> stars = {
        {1, 0.0, 0.0, 100.0, 3.0},       // Top-left corner
        {2, 99.0, 99.0, 100.0, 3.0}      // Bottom-right corner
    };

    DistanceTransform dt;
    dt.compute(stars, w, h);

    EXPECT_EQ(dt.at(0, 0), 0u);
    EXPECT_EQ(dt.at(99, 99), 0u);
    EXPECT_EQ(dt.at(1, 0), 1u);
    EXPECT_EQ(dt.at(0, 1), 1u);
    EXPECT_EQ(dt.at(98, 99), 1u);
}

// Verifies 100% pixel-by-pixel equivalence between Maurer's linear O(W*H) algorithm and brute-force O(W*H*N)
TEST(DistanceTransformTest, CompareAgainstBruteForce) {
    int w = 80, h = 80;
    std::vector<ImageStar> stars = {
        {1, 10.0, 15.0, 100.0, 2.0},
        {2, 45.0, 20.0, 100.0, 2.5},
        {3, 70.0, 65.0, 100.0, 3.0},
        {4, 25.0, 75.0, 100.0, 3.5},
        {5, 40.0, 40.0, 100.0, 4.0}
    };

    DistanceTransform dt_linear;
    dt_linear.compute(stars, w, h);

    DistanceTransform dt_brute;
    dt_brute.compute_brute_force(stars, w, h);

    // Check 100% of all pixels for exact equality
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            uint32_t val_linear = dt_linear.at(x, y);
            uint32_t val_brute = dt_brute.at(x, y);
            ASSERT_EQ(val_linear, val_brute)
                << "Mismatch at pixel (" << x << ", " << y << "): linear="
                << val_linear << " brute=" << val_brute;
        }
    }
}
