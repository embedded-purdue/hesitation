/**
 * @file test_camera_model.cpp
 * @brief Unit tests for pinhole camera projection, FOV bounds, and lens distortion.
 *
 * @reference
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." JGCD, 36(6), pp. 1672–1680.
 *   Section II.A "Pinhole Model and Camera Distortions", Eqs. (3–8). DOI: 10.2514/1.61082.
 */

#include <gtest/gtest.h>
#include "camera/camera_model.hpp"
#include <cmath>

using namespace star_tracker;

// Verifies optical boresight maps directly to sensor principal point (cx, cy)
TEST(CameraModelTest, PrincipalPoint) {
    double fov_rad = 20.0 * M_PI / 180.0;
    CameraModel cam = CameraModel::from_fov(1024, 1024, fov_rad, fov_rad);

    Vector3 boresight(0.0, 0.0, 1.0);
    double px = 0.0, py = 0.0;
    EXPECT_TRUE(cam.vector_to_pixel(boresight, px, py));
    EXPECT_NEAR(px, 512.0, 1e-9);
    EXPECT_NEAR(py, 512.0, 1e-9);
}

// Verifies round-trip consistency: pixel -> unit vector -> pixel
TEST(CameraModelTest, RoundTripProjection) {
    double fov_rad = 20.0 * M_PI / 180.0;
    CameraModel cam = CameraModel::from_fov(1024, 1024, fov_rad, fov_rad);

    double test_x = 350.25;
    double test_y = 780.75;

    Vector3 ray = cam.pixel_to_vector(test_x, test_y);
    double proj_x = 0.0, proj_y = 0.0;
    EXPECT_TRUE(cam.vector_to_pixel(ray, proj_x, proj_y));

    EXPECT_NEAR(proj_x, test_x, 1e-6);
    EXPECT_NEAR(proj_y, test_y, 1e-6);
}

// Verifies sensor boundary gating and backward-pointing ray rejection
TEST(CameraModelTest, BoundaryAndFov) {
    CameraModel cam = CameraModel::from_fov(1024, 1024, 0.35, 0.35);

    EXPECT_TRUE(cam.is_in_bounds(0.0, 0.0));
    EXPECT_TRUE(cam.is_in_bounds(1023.9, 1023.9));
    EXPECT_FALSE(cam.is_in_bounds(-1.0, 500.0));
    EXPECT_FALSE(cam.is_in_bounds(500.0, 1024.0));

    // Behind camera (+Z < 0)
    Vector3 behind(0.0, 0.0, -1.0);
    double x = 0.0, y = 0.0;
    EXPECT_FALSE(cam.vector_to_pixel(behind, x, y));
}

// Verifies 5th-order radial lens distortion forward apply and iterative remove round-trip
TEST(CameraModelTest, DistortionModel) {
    DistortionModel dist(-0.05, 0.01);
    double xn = 0.1, yn = 0.15;
    double xd = 0.0, yd = 0.0;
    dist.apply(xn, yn, xd, yd);

    double x_rec = 0.0, y_rec = 0.0;
    dist.remove(xd, yd, x_rec, y_rec);

    EXPECT_NEAR(x_rec, xn, 1e-6);
    EXPECT_NEAR(y_rec, yn, 1e-6);
}
