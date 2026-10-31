/**
 * @file test_centroid_matcher.cpp
 * @brief Unit tests for Delabie's centroid method, 2D rigid transformation, and 6-feature filter.
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." JGCD, 36(2), pp. 476–488. Section II.C.2
 *   "Centroid Method", Section II.E.1 "Discarding False Images". DOI: 10.2514/1.56860.
 */

#include <gtest/gtest.h>
#include "lost_in_space/centroid_matcher.hpp"
#include "lost_in_space/candidate_filter.hpp"
#include <cmath>

using namespace star_tracker;

// Verifies translation-only alignment recovery (zero rotation)
TEST(CentroidMatcherTest, TranslationOnly) {
    std::vector<ImageStar> ref_triplet = {
        {1, 100.0, 100.0, 100.0, 2.0},
        {2, 150.0, 110.0, 100.0, 2.5},
        {3, 120.0, 180.0, 100.0, 3.0}
    };
    CentroidFeatures ref_feat = CentroidFeatures::compute(ref_triplet);
    ASSERT_TRUE(ref_feat.valid);

    // Apply translation: tx = 25.0, ty = -40.0
    double tx = 25.0, ty = -40.0;
    std::vector<ImageStar> cam_triplet = {
        {1, 100.0 + tx, 100.0 + ty, 100.0, 2.0},
        {2, 150.0 + tx, 110.0 + ty, 100.0, 2.5},
        {3, 120.0 + tx, 180.0 + ty, 100.0, 3.0}
    };
    CentroidFeatures cam_feat = CentroidFeatures::compute(cam_triplet);
    ASSERT_TRUE(cam_feat.valid);

    CandidateFilter filter;
    EXPECT_TRUE(filter.matches(cam_feat, ref_feat));

    Alignment align = CentroidMatcher::compute_alignment(cam_feat, ref_feat);
    EXPECT_TRUE(align.valid);
    EXPECT_NEAR(align.rotation, 0.0, 1e-6);
    EXPECT_NEAR(align.translation_x, tx, 1e-6);
    EXPECT_NEAR(align.translation_y, ty, 1e-6);
}

// Verifies simultaneous recovery of roll rotation angle phi and translations (tx, ty)
TEST(CentroidMatcherTest, CombinedRotationAndTranslation) {
    std::vector<ImageStar> ref_triplet = {
        {1, 500.0, 500.0, 100.0, 2.0},
        {2, 560.0, 520.0, 100.0, 2.5},
        {3, 490.0, 580.0, 100.0, 3.0}
    };
    CentroidFeatures ref_feat = CentroidFeatures::compute(ref_triplet);
    ASSERT_TRUE(ref_feat.valid);

    double rot = 0.45; // ~25.8 degrees
    double tx = 30.0, ty = -20.0;
    double c = std::cos(rot), s = std::sin(rot);

    std::vector<ImageStar> cam_triplet;
    for (const auto& rs : ref_triplet) {
        double xc = rs.x * c - rs.y * s + tx;
        double yc = rs.x * s + rs.y * c + ty;
        cam_triplet.emplace_back(rs.catalog_id, xc, yc, rs.brightness, rs.magnitude);
    }

    CentroidFeatures cam_feat = CentroidFeatures::compute(cam_triplet);
    ASSERT_TRUE(cam_feat.valid);

    CandidateFilter filter;
    EXPECT_TRUE(filter.matches(cam_feat, ref_feat));

    Alignment align = CentroidMatcher::compute_alignment(cam_feat, ref_feat);
    EXPECT_TRUE(align.valid);
    EXPECT_NEAR(align.rotation, rot, 1e-5);
    EXPECT_NEAR(align.translation_x, tx, 1e-5);
    EXPECT_NEAR(align.translation_y, ty, 1e-5);

    // Verify all 3 points transform accurately onto camera positions
    for (size_t i = 0; i < 3; ++i) {
        double xc = 0.0, yc = 0.0;
        align.transform_point(ref_triplet[i].x, ref_triplet[i].y, xc, yc);
        EXPECT_NEAR(xc, cam_triplet[i].x, 1e-5);
        EXPECT_NEAR(yc, cam_triplet[i].y, 1e-5);
    }
}
