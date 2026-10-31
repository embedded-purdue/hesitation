/**
 * @file test_aim.cpp
 * @brief Unit tests for closed-form Attitude Estimation using Image Matching (AIM).
 *
 * @reference
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." Journal of Guidance, Control,
 *   and Dynamics, 36(6), pp. 1672–1680. Section II.D "Attitude-Estimation Algorithm",
 *   Eqs. (9–21), Section III.B "Accuracy". DOI: 10.2514/1.61082.
 */

#include <gtest/gtest.h>
#include "aim/aim_estimator.hpp"
#include "camera/camera_model.hpp"
#include <cmath>

using namespace star_tracker;

// Verifies closed-form recovery of pure 2D focal-plane translations (tx, ty) with zero rotation
TEST(AIMTest, PureTranslation) {
    CameraModel cam = CameraModel::from_fov(1024, 1024, 0.35, 0.35);
    AIMEstimator aim(cam);

    std::vector<ImageStar> ref_stars = {
        {1, 300.0, 300.0, 100.0, 2.0},
        {2, 600.0, 350.0, 100.0, 2.5},
        {3, 450.0, 700.0, 100.0, 3.0},
        {4, 700.0, 650.0, 100.0, 3.5}
    };

    double tx_true = 12.5;
    double ty_true = -8.2;

    std::vector<ImageStar> obs_stars;
    std::vector<double> weights(ref_stars.size(), 1.0);
    for (const auto& s : ref_stars) {
        obs_stars.emplace_back(s.catalog_id, s.x + tx_true, s.y + ty_true, s.brightness, s.magnitude);
    }

    AIMResult res = aim.match_paired_stars(obs_stars, ref_stars, weights, Quaternion::identity());

    EXPECT_TRUE(res.valid);
    EXPECT_NEAR(res.image_rotation, 0.0, 1e-7);
    EXPECT_NEAR(res.translation_x, tx_true, 1e-7);
    EXPECT_NEAR(res.translation_y, ty_true, 1e-7);
    EXPECT_NEAR(res.rms_error, 0.0, 1e-7);
}

// Verifies closed-form recovery of pure 2D focal-plane roll rotation angle phi (Eq. 12)
TEST(AIMTest, PureRotation) {
    CameraModel cam = CameraModel::from_fov(1024, 1024, 0.35, 0.35);
    AIMEstimator aim(cam);

    std::vector<ImageStar> ref_stars = {
        {1, 400.0, 400.0, 100.0, 2.0},
        {2, 650.0, 420.0, 100.0, 2.5},
        {3, 500.0, 750.0, 100.0, 3.0},
        {4, 600.0, 600.0, 100.0, 3.5}
    };

    double phi_true = 0.035; // ~2 degrees
    double c = std::cos(phi_true);
    double s = std::sin(phi_true);

    // Transform: x' = hx*cos + hy*sin, y' = -hx*sin + hy*cos
    std::vector<ImageStar> obs_stars;
    std::vector<double> weights(ref_stars.size(), 1.0);
    for (const auto& rs : ref_stars) {
        double xp =  rs.x * c + rs.y * s;
        double yp = -rs.x * s + rs.y * c;
        obs_stars.emplace_back(rs.catalog_id, xp, yp, rs.brightness, rs.magnitude);
    }

    AIMResult res = aim.match_paired_stars(obs_stars, ref_stars, weights, Quaternion::identity());

    EXPECT_TRUE(res.valid);
    EXPECT_NEAR(res.image_rotation, phi_true, 1e-6);
    EXPECT_NEAR(res.rms_error, 0.0, 1e-6);
}

// Verifies AIM attitude refinement achieves sub-arcsecond (< 1.0 arcsec) pointing accuracy
TEST(AIMTest, AttitudeCorrectionAccuracy) {
    CameraModel cam = CameraModel::from_fov(1024, 1024, 0.35, 0.35);
    AIMEstimator aim(cam);

    ReferenceImage ref;
    ref.attitude = Quaternion::identity();
    ref.stars = {
        {1, 350.0, 350.0, 100.0, 2.0},
        {2, 650.0, 380.0, 100.0, 2.5},
        {3, 400.0, 680.0, 100.0, 3.0},
        {4, 620.0, 640.0, 100.0, 3.5},
        {5, 510.0, 520.0, 100.0, 4.0}
    };

    // 100 arcseconds rotation around boresight and 50 arcseconds around cross-axis
    double arcsec_to_rad = M_PI / (180.0 * 3600.0);
    Quaternion true_dq = Quaternion::from_euler(50.0 * arcsec_to_rad, 40.0 * arcsec_to_rad, 100.0 * arcsec_to_rad);
    Quaternion true_att = true_dq * ref.attitude;

    // Project reference stars under true attitude
    std::vector<ImageStar> obs_stars;
    for (const auto& rs : ref.stars) {
        Vector3 v_ref = cam.pixel_to_vector(rs.x, rs.y);
        Vector3 v_cam = true_dq.rotate(v_ref);
        double px = 0.0, py = 0.0;
        cam.vector_to_pixel(v_cam, px, py);
        obs_stars.emplace_back(rs.catalog_id, px, py, rs.brightness, rs.magnitude);
    }

    // Give coarse attitude = ref.attitude (difference ~100 arcsec)
    AIMResult res = aim.estimate(obs_stars, ref, ref.attitude);

    EXPECT_TRUE(res.valid);
    double err = Quaternion::angular_distance(res.corrected_attitude, true_att);
    EXPECT_LT(err * 180.0 / M_PI * 3600.0, 1.0); // Sub-arcsecond accuracy!
}
