/**
 * @file test_centroid_extractor.cpp
 * @brief Unit tests for subpixel centroid extraction accuracy and connected-component detection.
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." JGCD, 36(2), pp. 476–488. Section III.A
 *   "Positional Error". DOI: 10.2514/1.56860.
 */

#include <gtest/gtest.h>
#include "camera/camera_model.hpp"
#include "detection/centroid_extractor.hpp"
#include "detection/image.hpp"
#include "detection/star_detector.hpp"

using namespace star_tracker;

// Verifies subpixel Center-of-Gravity recovers the exact center of a symmetric spot
TEST(CentroidExtractorTest, SymmetricSpot) {
    // 3x3 symmetric star spot centered at (10, 10)
    std::vector<PixelCoord> spot = {
        {9, 9, 50.0},  {10, 9, 100.0},  {11, 9, 50.0},
        {9, 10, 100.0}, {10, 10, 200.0}, {11, 10, 100.0},
        {9, 11, 50.0}, {10, 11, 100.0}, {11, 11, 50.0}
    };

    ImageStar star = CentroidExtractor::extract_centroid(spot, 0.0);
    EXPECT_NEAR(star.x, 10.0, 1e-9);
    EXPECT_NEAR(star.y, 10.0, 1e-9);
    EXPECT_GT(star.brightness, 0.0);
}

// Verifies subpixel interpolation responds correctly to asymmetrical pixel flux distribution
TEST(CentroidExtractorTest, SubpixelShift) {
    // Star shifted slightly towards (10.2, 10.0)
    std::vector<PixelCoord> spot = {
        {9, 10, 80.0}, {10, 10, 200.0}, {11, 10, 120.0}
    };

    ImageStar star = CentroidExtractor::extract_centroid(spot, 0.0);
    EXPECT_GT(star.x, 10.0);
    EXPECT_NEAR(star.y, 10.0, 1e-9);
}

// Verifies end-to-end connected component detection on synthetic Gaussian star spots with noise
TEST(StarDetectorTest, DetectFromSyntheticImage) {
    CameraModel cam = CameraModel::from_fov(256, 256, 0.35, 0.35);

    std::vector<ImageStar> true_stars = {
        {1, 50.0, 50.0, 1000.0, 3.0},
        {2, 150.0, 180.0, 800.0, 3.5},
        {3, 200.0, 70.0, 500.0, 4.0}
    };

    Image img = Image::render_star_field(256, 256, true_stars, 1.0, 0.5, 5);

    StarDetectorConfig cfg;
    cfg.adaptive_threshold = true;
    cfg.min_region_size = 2;
    cfg.min_total_flux = 20.0;
    StarDetector detector(cfg);

    std::vector<ImageStar> detected = detector.detect_stars(img, cam);

    EXPECT_GE(detected.size(), 3u);

    // Verify each true star has a corresponding detected star within 0.5 pixels
    for (const auto& ts : true_stars) {
        bool found = false;
        for (const auto& ds : detected) {
            double dx = ds.x - ts.x;
            double dy = ds.y - ts.y;
            if (std::sqrt(dx * dx + dy * dy) < 0.5) {
                found = true;
                break;
            }
        }
        EXPECT_TRUE(found) << "Failed to detect star at (" << ts.x << ", " << ts.y << ")";
    }
}
