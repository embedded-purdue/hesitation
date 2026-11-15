
/**
 * @file test_aim.cpp
 * @brief Unit tests for closed-form Attitude Estimation using Image Matching
 * (AIM).
 *
 * @author Eric Yu
 * @date October 2026
 * @version 1.0.0
 *
 * @reference
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013).
 *   "Highly Efficient Attitude-Estimation Algorithm for Star Trackers
 *   Using Optimal Image Matching."
 *   Journal of Guidance, Control, and Dynamics, 36(6), pp. 1672–1680.
 *   Section II.D, Eqs. (9–21).
 *   DOI: 10.2514/1.61082.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "aim/aim_estimator.hpp"
#include "camera/camera_model.hpp"

using namespace star_tracker;

// Tests recovery of pure image translation with zero rotation.
TEST(AIMTest, PureTranslation) {
  CameraModel cam = CameraModel::from_fov(1024, 1024, 0.35, 0.35);

  AIMEstimator aim(cam);

  std::vector<ImageStar> ref_stars = {{1, 300.0, 300.0, 100.0, 2.0},
                                      {2, 600.0, 350.0, 100.0, 2.5},
                                      {3, 450.0, 700.0, 100.0, 3.0},
                                      {4, 700.0, 650.0, 100.0, 3.5}};

  const double tx_true = 12.5;
  const double ty_true = -8.2;

  std::vector<ImageStar> obs_stars;
  std::vector<double> weights(ref_stars.size(), 1.0);

  for (const auto &rs : ref_stars) {
    double xp = rs.x + tx_true;
    double yp = rs.y + ty_true;

    obs_stars.emplace_back(rs.catalog_id, xp, yp, rs.brightness, rs.magnitude);
  }

  AIMResult res = aim.match_paired_stars(obs_stars, ref_stars, weights,
                                         Quaternion::identity());

  EXPECT_TRUE(res.valid);
  EXPECT_NEAR(res.image_rotation, 0.0, 1e-7);
  EXPECT_NEAR(res.translation_x, tx_true, 1e-7);
  EXPECT_NEAR(res.translation_y, ty_true, 1e-7);
  EXPECT_NEAR(res.rms_error, 0.0, 1e-7);
}

// Tests recovery of a pure 2D image rotation using the active
// rotation convention:
//
// x' = hx*cos(phi) - hy*sin(phi)
// y' = hx*sin(phi) + hy*cos(phi)
TEST(AIMTest, PureRotation) {
  CameraModel cam = CameraModel::from_fov(1024, 1024, 0.35, 0.35);

  AIMEstimator aim(cam);

  std::vector<ImageStar> ref_stars = {{1, 400.0, 400.0, 100.0, 2.0},
                                      {2, 650.0, 420.0, 100.0, 2.5},
                                      {3, 500.0, 750.0, 100.0, 3.0},
                                      {4, 600.0, 600.0, 100.0, 3.5}};

  const double phi_true = 0.035;
  const double c = std::cos(phi_true);
  const double s = std::sin(phi_true);

  std::vector<ImageStar> obs_stars;
  std::vector<double> weights(ref_stars.size(), 1.0);

  for (const auto &rs : ref_stars) {
    double xp = rs.x * c - rs.y * s;
    double yp = rs.x * s + rs.y * c;

    obs_stars.emplace_back(rs.catalog_id, xp, yp, rs.brightness, rs.magnitude);
  }

  AIMResult res = aim.match_paired_stars(obs_stars, ref_stars, weights,
                                         Quaternion::identity());

  EXPECT_TRUE(res.valid);
  EXPECT_NEAR(res.image_rotation, phi_true, 1e-6);
  EXPECT_NEAR(res.rms_error, 0.0, 1e-6);
}

// Tests whether AIM can refine a coarse attitude to recover
// a known small 3D attitude correction.
TEST(AIMTest, AttitudeCorrectionAccuracy) {
  CameraModel cam = CameraModel::from_fov(1024, 1024, 0.35, 0.35);

  AIMEstimator aim(cam);

  ReferenceImage ref;
  ref.attitude = Quaternion::identity();

  ref.stars = {{1, 350.0, 350.0, 100.0, 2.0},
               {2, 650.0, 380.0, 100.0, 2.5},
               {3, 400.0, 680.0, 100.0, 3.0},
               {4, 620.0, 640.0, 100.0, 3.5},
               {5, 510.0, 520.0, 100.0, 4.0}};

  // Generate a small known attitude correction.
  const double arcsec_to_rad = M_PI / (180.0 * 3600.0);

  Quaternion true_dq = Quaternion::from_euler(
      50.0 * arcsec_to_rad, 40.0 * arcsec_to_rad, 100.0 * arcsec_to_rad);

  Quaternion true_att = true_dq * ref.attitude;

  // Project reference stars under the corrected attitude.
  std::vector<ImageStar> obs_stars;

  for (const auto &rs : ref.stars) {
    Vector3 v_ref = cam.pixel_to_vector(rs.x, rs.y);
    Vector3 v_cam = true_dq.rotate(v_ref);

    double px = 0.0;
    double py = 0.0;

    cam.vector_to_pixel(v_cam, px, py);

    obs_stars.emplace_back(rs.catalog_id, px, py, rs.brightness, rs.magnitude);
  }

  // The estimator receives the original coarse attitude.
  AIMResult res = aim.estimate(obs_stars, ref, ref.attitude);

  EXPECT_TRUE(res.valid);

  const double err =
      Quaternion::angular_distance(res.corrected_attitude, true_att);

  const double error_arcsec = err * 180.0 / M_PI * 3600.0;
  std::cout << std::fixed << std::setprecision(12);

  std::cout << "\nAIM diagnostics:\n";
  std::cout << "Image rotation: " << res.image_rotation << " rad\n";
  std::cout << "Translation X:  " << res.translation_x << " px\n";
  std::cout << "Translation Y:  " << res.translation_y << " px\n";
  std::cout << "RMS error:      " << res.rms_error << " px\n";

  std::cout << "\nAttitude comparison:\n";
  std::cout << "Angular error:  " << error_arcsec << " arcsec\n";
  EXPECT_LT(error_arcsec, 1.0)
      << "Attitude error: " << error_arcsec << " arcseconds";
}
