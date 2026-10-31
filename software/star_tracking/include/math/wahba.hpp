/**
 * @file wahba.hpp
 * @brief Solution of Wahba's Problem using Shuster's QUEST (Quaternion Estimator) algorithm.
 *
 * @details
 * Wahba's problem (Wahba, 1965) seeks the proper orthogonal rotation matrix A in SO(3) that
 * minimizes the weighted sum of squared Euclidean vector residuals:
 *   L(A) = 0.5 * sum_{i=1}^N a_i * || b_i - A * r_i ||^2
 * where:
 *   - b_i: Unit direction vectors observed in the sensor (camera) frame.
 *   - r_i: Known catalog reference unit vectors in the celestial inertial frame (ECI J2000).
 *   - a_i: Non-negative measurement weights (sum(a_i) = 1 or total weight).
 *
 * In Davenport's formulation (1968), Wahba's problem is recast into maximizing the bilinear form:
 *   g(q) = q^T * K * q
 * where q is the attitude quaternion and K is Davenport's 4x4 symmetric matrix:
 *   K = [ sigma       Z^T     ]
 *       [   Z     S - sigma*I ]
 * with:
 *   - B = sum_{i=1}^N a_i * b_i * r_i^T  (Davenport's attitude profile matrix)
 *   - S = B + B^T
 *   - sigma = tr(B)
 *   - Z = [ B(1,2) - B(2,1), B(2,0) - B(0,2), B(0,1) - B(1,0) ]^T = sum a_i * (b_i x r_i)
 *
 * Shuster's QUEST algorithm (Shuster & Oh, 1981) observes that for star trackers with high-accuracy
 * measurements (centroiding noise on the order of arcseconds), the optimal eigenvalue lambda_opt
 * is extraordinarily close to the sum of weights:
 *   lambda_opt ≈ sum_{i=1}^N a_i
 * This yields the optimal Rodrigues Gibbs vector p in closed form:
 *   p = [ (lambda_opt + sigma)*I - S ]^-1 * Z
 * and the corresponding normalized optimal attitude quaternion:
 *   q_opt = 1 / sqrt(1 + ||p||^2) * [ 1, p^T ]^T
 *
 * Role in Star Tracker Pipeline:
 * - Used during ACQUISITION / RECOVERY (Lost-in-Space) to obtain the initial 3-axis attitude
 *   estimate once candidate star pairs are verified via the Shortest Distance Transform.
 * - Seeds the AIM (Attitude Estimation using Image Matching) algorithm (Delabie et al., 2013).
 *
 * @reference
 * - Shuster, M. D., and Oh, S. D. (1981). "Three-Axis Attitude Determination from Vector Observations."
 *   Journal of Guidance and Control, 4(1), pp. 70–77. DOI: 10.2514/3.19717.
 * - Wahba, G. (1965). "A Least Squares Estimate of Spacecraft Attitude." SIAM Review, 7(3), p. 409.
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." JGCD, 36(2), pp. 476–488. DOI: 10.2514/1.56860.
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." JGCD, 36(6), pp. 1672–1680.
 */

#pragma once

#include "math/matrix.hpp"
#include "math/quaternion.hpp"
#include "math/vector.hpp"
#include <cmath>
#include <vector>

namespace star_tracker {

/**
 * @brief Solves Wahba's problem using Shuster's QUEST algorithm.
 *
 * @details
 * Computes the optimal unit quaternion q representing the attitude rotation mapping celestial
 * inertial star vectors v_ref_i to observed camera rays w_cam_i:
 *   w_cam_i = R(q) * v_ref_i
 *
 * Algorithm Steps:
 * 1. Accumulate Davenport's attitude profile matrix B = sum w_i * (w_cam_i * v_ref_i^T).
 * 2. Form symmetric matrix S = B + B^T and scalar trace sigma = tr(B).
 * 3. Compute vector Z from the anti-symmetric components of B: Z_k = epsilon_{ijk} B_{ij}.
 * 4. Approximate lambda_max ≈ total_weight = sum w_i.
 * 5. Solve linear system M * x_crf = Z, where M = (lambda_max + sigma)*I - S.
 * 6. Construct and normalize the resulting quaternion q.
 *
 * @param cam_rays Observed star unit vectors in camera frame (w_cam). Must have at least 2 non-collinear rays.
 * @param ref_vectors Corresponding star unit vectors in celestial inertial frame (v_ref). Must match size of cam_rays.
 * @param weights Optional measurement weights (e.g. based on star brightness or centroid variance). If empty, uniform weights (1.0) are used.
 * @return Optimal unit quaternion q (inertial to camera). Returns identity if fewer than 2 stars or singular.
 */
inline Quaternion solve_quest(
    const std::vector<Vector3>& cam_rays,
    const std::vector<Vector3>& ref_vectors,
    const std::vector<double>& weights = {}
) {
    size_t n = cam_rays.size();
    if (n < 2 || ref_vectors.size() != n) {
        return Quaternion::identity();
    }

    // Step 1: Accumulate Davenport's attitude profile matrix B and total weight
    Matrix3 B = Matrix3::zeros();
    double total_w = 0.0;
    for (size_t i = 0; i < n; ++i) {
        double w = weights.empty() ? 1.0 : weights[i];
        total_w += w;
        const auto& wc = cam_rays[i];
        const auto& vi = ref_vectors[i];

        // Outer product B += w * (wc * vi^T)
        B(0, 0) += w * wc.x * vi.x;
        B(0, 1) += w * wc.x * vi.y;
        B(0, 2) += w * wc.x * vi.z;

        B(1, 0) += w * wc.y * vi.x;
        B(1, 1) += w * wc.y * vi.y;
        B(1, 2) += w * wc.y * vi.z;

        B(2, 0) += w * wc.z * vi.x;
        B(2, 1) += w * wc.z * vi.y;
        B(2, 2) += w * wc.z * vi.z;
    }

    if (total_w <= 1e-12) return Quaternion::identity();

    // Step 2 & 3: Compute S, sigma, and Z
    Matrix3 S = B + B.transpose();
    double sigma = B(0, 0) + B(1, 1) + B(2, 2);
    Vector3 Z(B(1, 2) - B(2, 1),
              B(2, 0) - B(0, 2),
              B(0, 1) - B(1, 0));

    // Step 4: For high-accuracy star measurements, lambda_max is exceptionally close to total_w
    double lambda = total_w;
    Matrix3 M = Matrix3::identity() * (lambda + sigma) - S;

    // Step 5 & 6: Invert M to find Gibbs vector x_crf and reconstruct quaternion
    try {
        Vector3 x_crf = M.inverse() * Z;
        double gamma = 1.0 / std::sqrt(1.0 + x_crf.norm_squared());
        // Hamilton quaternion mapping inertial to body: [gamma, -gamma * x_crf]
        return Quaternion(gamma, -gamma * x_crf.x, -gamma * x_crf.y, -gamma * x_crf.z).normalized();
    } catch (const std::exception&) {
        // Fallback for ill-conditioned geometry (collinear stars or near 180 deg ambiguity)
        return Quaternion::identity();
    }
}

} // namespace star_tracker
