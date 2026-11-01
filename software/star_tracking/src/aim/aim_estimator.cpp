/**
 * @file aim_estimator.cpp
 * @brief Implementation of Attitude Estimation using Image Matching (AIM) closed-form solver.
 *
 * @author Eric Yu
 * @date October 2026
 * @version 1.0.0
 *
 * @reference
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." Journal of Guidance, Control,
 *   and Dynamics, 36(6), pp. 1672–1680. DOI: 10.2514/1.61082.
 */

#include "aim/aim_estimator.hpp"
#include <cmath>

namespace star_tracker {

AIMResult
AIMEstimator::match_paired_stars(const std::vector<ImageStar> &observed_stars,
                                 const std::vector<ImageStar> &reference_stars,
                                 const std::vector<double> &weights,
                                 const Quaternion &coarse_attitude) {
  AIMResult res;
  size_t n = observed_stars.size();

  if (n < 2 || reference_stars.size() != n || weights.size() != n) {
    res.valid = false;
    return res;
  }

  double sx = 0.0;
  double sy = 0.0;

  double sx_hat = 0.0;
  double sy_hat = 0.0;

  double sxx_hat = 0.0;
  double syy_hat = 0.0;

  double sxy_hat = 0.0;
  double syx_hat = 0.0;

  double weight_sum = 0.0;
  for (const double w : weights) {
    if (!std::isfinite(w) || w <= 0.0) {
      res.valid = false;
      return res;
    }
    weight_sum += w;
  }

  for (size_t i = 0; i < n; ++i) {
    double w = weights[i] / weight_sum;

    double x = observed_stars[i].x;
    double y = observed_stars[i].y;

    double x_hat = reference_stars[i].x;
    double y_hat = reference_stars[i].y;

    sx += w * x;
    sy += w * y;

    sx_hat += w * x_hat;
    sy_hat += w * y_hat;

    sxx_hat += w * x * x_hat;
    syy_hat += w * y * y_hat;

    sxy_hat += w * x * y_hat;
    syx_hat += w * y * x_hat;
  }

  double numerator = sx * sy_hat - sy * sx_hat - sxy_hat + syx_hat;
  double denominator = -sx * sx_hat - sy * sy_hat + sxx_hat + syy_hat;

  double phi = std::atan2(numerator, denominator);
  double cos_phi = std::cos(phi);
  double sin_phi = std::sin(phi);

  double tx = sx - sx_hat * cos_phi + sy_hat * sin_phi;
  double ty = sy - sx_hat * sin_phi - sy_hat * cos_phi;

  double px = camera_.principal_x;
  double py = camera_.principal_y;

  double tu = tx + (cos_phi - 1.0) * px - sin_phi * py;
  double tv = ty + sin_phi * px + (cos_phi - 1.0) * py;

  double cost = 0.0;
  for (size_t i = 0; i < n; ++i) {
    double w = weights[i] / weight_sum;
    double hx = reference_stars[i].x;
    double hy = reference_stars[i].y;

    double x_prime = hx * cos_phi - hy * sin_phi + tx;
    double y_prime = hx * sin_phi + hy * cos_phi + ty;

    double ex = observed_stars[i].x - x_prime;
    double ey = observed_stars[i].y - y_prime;

    cost += w * (ex * ex + ey * ey);
  }

  double rms = std::sqrt(cost);

  double f = camera_.focal_length;
  double theta_x = -tv / f;
  double theta_y = tu / f;
  double theta_z = phi;

  Vector3 rot_vec(theta_x, theta_y, theta_z);
  Quaternion q_diff = Quaternion::from_rotation_vector(rot_vec);
  Quaternion q_corrected = (q_diff * coarse_attitude).normalized();

  res.corrected_attitude = q_corrected;
  res.image_rotation = phi;
  res.translation_x = tx;
  res.translation_y = ty;
  res.cost = cost;
  res.rms_error = rms;
  res.num_matched_stars = static_cast<int>(n);

  res.valid = (res.num_matched_stars >= config_.min_matched_stars) &&
              (res.rms_error <= config_.max_rms_error);

  return res;
}

AIMResult AIMEstimator::estimate(const std::vector<ImageStar> &observed_stars,
                                 const ReferenceImage &reference,
                                 const Quaternion &coarse_attitude) {
  AIMResult res;
  res.valid = false;

  if (observed_stars.empty() || reference.stars.empty()) {
    return res;
  }

  std::vector<ImageStar> matched_obs;
  std::vector<ImageStar> matched_ref;
  std::vector<double> weights;

  std::vector<bool> obs_used(observed_stars.size(), false);
  double radius_sq = config_.match_radius_pixels * config_.match_radius_pixels;

  for (const auto &rstar : reference.stars) {
    int best_idx = -1;
    double best_dist_sq = radius_sq;

    for (size_t j = 0; j < observed_stars.size(); ++j) {
      if (obs_used[j])
        continue;

      double dx = observed_stars[j].x - rstar.x;
      double dy = observed_stars[j].y - rstar.y;
      double d2 = dx * dx + dy * dy;

      if (d2 < best_dist_sq) {
        best_dist_sq = d2;
        best_idx = static_cast<int>(j);
      }
    }

    if (best_idx >= 0) {
      obs_used[best_idx] = true;
      matched_obs.push_back(observed_stars[best_idx]);
      matched_ref.push_back(rstar);
      weights.push_back(1.0);
    }
  }

  if (static_cast<int>(matched_obs.size()) < config_.min_matched_stars) {
    res.num_matched_stars = static_cast<int>(matched_obs.size());
    return res;
  }

  return match_paired_stars(matched_obs, matched_ref, weights, coarse_attitude);
}

} // namespace star_tracker
