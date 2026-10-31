/**
 * @file aim_estimator.cpp
 * @brief Implementation of Delabie's closed-form Attitude Estimation using Image Matching (AIM).
 *
 * @reference
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." Journal of Guidance, Control,
 *   and Dynamics, 36(6), pp. 1672–1680. Section II.D "Attitude-Estimation Algorithm",
 *   Eqs. (9–21). DOI: 10.2514/1.61082.
 */

#include "aim/aim_estimator.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace star_tracker {

AIMResult AIMEstimator::match_paired_stars(
    const std::vector<ImageStar>& observed_stars,
    const std::vector<ImageStar>& reference_stars,
    const std::vector<double>& weights,
    const Quaternion& coarse_attitude
) {
    AIMResult res;
    size_t n = observed_stars.size();
    if (n < 2 || reference_stars.size() != n || weights.size() != n) {
        res.valid = false;
        return res;
    }

    // Step 1: Accumulate total weight and weighted first moments (Eq. 15)
    double total_w = 0.0;
    double sum_x = 0.0, sum_y = 0.0;
    double sum_hx = 0.0, sum_hy = 0.0;

    for (size_t i = 0; i < n; ++i) {
        double w = weights[i];
        total_w += w;
        sum_x += w * observed_stars[i].x;
        sum_y += w * observed_stars[i].y;
        sum_hx += w * reference_stars[i].x;
        sum_hy += w * reference_stars[i].y;
    }

    if (total_w <= 1e-12) {
        res.valid = false;
        return res;
    }

    // Centroid of observed camera stars (x_bar, y_bar) and reference database stars (x_hat_bar, y_hat_bar)
    double mean_x = sum_x / total_w;
    double mean_y = sum_y / total_w;
    double mean_hx = sum_hx / total_w;
    double mean_hy = sum_hy / total_w;

    // Step 2: Accumulate centered cross-correlation terms C and S (Eq. 15)
    double C = 0.0;
    double S = 0.0;

    for (size_t i = 0; i < n; ++i) {
        double w = weights[i];
        double dx = observed_stars[i].x - mean_x;
        double dy = observed_stars[i].y - mean_y;
        double dhx = reference_stars[i].x - mean_hx;
        double dhy = reference_stars[i].y - mean_hy;

        C += w * (dx * dhx + dy * dhy);
        S += w * (dx * dhy - dy * dhx);
    }

    // Step 3: Closed-form optimal rotation angle phi (Delabie et al., 2013, Eq. 12)
    double phi = std::atan2(S, C);
    double cos_phi = std::cos(phi);
    double sin_phi = std::sin(phi);

    // Step 4: Closed-form optimal translations tx, ty in raw pixel coordinates (Eqs. 13–14)
    double tx = mean_x - mean_hx * cos_phi - mean_hy * sin_phi;
    double ty = mean_y + mean_hx * sin_phi - mean_hy * cos_phi;

    // Step 5: Optical translation (tu, tv) centered at principal point (cx, cy)
    // Rotation occurs about the optical axis passing through (cx, cy)
    double cx = camera_.principal_x;
    double cy = camera_.principal_y;
    double u_mean = mean_x - cx;
    double v_mean = mean_y - cy;
    double hu_mean = mean_hx - cx;
    double hv_mean = mean_hy - cy;

    double tu = u_mean - hu_mean * cos_phi - hv_mean * sin_phi;
    double tv = v_mean + hu_mean * sin_phi - hv_mean * cos_phi;

    // Step 6: Calculate residual cost Gamma and RMS error (Eq. 9)
    double cost = 0.0;
    for (size_t i = 0; i < n; ++i) {
        double w = weights[i];
        double hx = reference_stars[i].x;
        double hy = reference_stars[i].y;

        double x_prime =  hx * cos_phi + hy * sin_phi + tx;
        double y_prime = -hx * sin_phi + hy * cos_phi + ty;

        double ex = observed_stars[i].x - x_prime;
        double ey = observed_stars[i].y - y_prime;

        cost += w * (ex * ex + ey * ey);
    }

    double rms = std::sqrt(cost / total_w);

    // Step 7: Differential attitude rotation vector in camera coordinates (Eqs. 16–18)
    // Small angle approximation: theta_x = -tv / F, theta_y = tu / F, theta_z = -phi
    double f = camera_.focal_length;
    double theta_x = -tv / f;
    double theta_y =  tu / f;
    double theta_z = -phi;

    Vector3 rot_vec(theta_x, theta_y, theta_z);
    // Construct small-angle rotation quaternion q_diff (Eq. 20)
    Quaternion q_diff = Quaternion::from_rotation_vector(rot_vec);

    // Step 8: Compose final attitude: q_e = q_diff ⊗ q_dat (Eq. 21)
    Quaternion q_corrected = (q_diff * coarse_attitude).normalized();

    res.corrected_attitude = q_corrected;
    res.image_rotation = phi;
    res.translation_x = tx;
    res.translation_y = ty;
    res.cost = cost;
    res.rms_error = rms;
    res.num_matched_stars = static_cast<int>(n);

    // Validate against acceptance thresholds
    res.valid = (res.num_matched_stars >= config_.min_matched_stars) &&
                (res.rms_error <= config_.max_rms_error);

    return res;
}

AIMResult AIMEstimator::estimate(
    const std::vector<ImageStar>& observed_stars,
    const ReferenceImage& reference,
    const Quaternion& coarse_attitude
) {
    AIMResult res;
    res.valid = false;

    if (observed_stars.empty() || reference.stars.empty()) {
        return res;
    }

    // Match each predicted reference star to the closest observed star within gating radius
    std::vector<ImageStar> matched_obs;
    std::vector<ImageStar> matched_ref;
    std::vector<double> weights;

    std::vector<bool> obs_used(observed_stars.size(), false);
    double radius_sq = config_.match_radius_pixels * config_.match_radius_pixels;

    for (const auto& rstar : reference.stars) {
        int best_idx = -1;
        double best_dist_sq = radius_sq;

        for (size_t j = 0; j < observed_stars.size(); ++j) {
            if (obs_used[j]) continue;

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

    // Execute closed-form AIM image matching on paired stars
    return match_paired_stars(matched_obs, matched_ref, weights, coarse_attitude);
}

} // namespace star_tracker
