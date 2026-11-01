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
 * @author Eric Yu
 * @date October 2026
 * @version 1.0.0
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
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
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
 * @param weights Optional measurement weights. If empty, uniform weights (1.0) are used.
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

/**
 * @brief Solves Wahba's problem using Davenport's q-method (Davenport, 1968).
 *
 * @details
 * Davenport's q-method finds the optimal attitude quaternion as the principal eigenvector
 * corresponding to the largest algebraic eigenvalue of the 4x4 symmetric matrix K:
 *   K = [ sigma       Z^T     ]
 *       [   Z     S - sigma*I ]
 *
 * The 4x4 symmetric eigensystem is solved unconditionally using cyclic Jacobi rotations.
 *
 * @reference
 * - Davenport, P. B. (1968). "A Vector Approach to the Algebra of Rotations with Applications."
 *   NASA Technical Note TN D-4696.
 * - Delabie et al. (2013). "Highly Efficient Attitude-Estimation Algorithm for Star Trackers
 *   Using Optimal Image Matching." JGCD, 36(6), p. 1673.
 *
 * @param cam_rays Observed star unit vectors in camera frame.
 * @param ref_vectors Corresponding star unit vectors in celestial inertial frame.
 * @param weights Optional weights per star measurement.
 * @return Optimal attitude quaternion (inertial to camera).
 */
inline Quaternion solve_q_davenport(
    const std::vector<Vector3>& cam_rays,
    const std::vector<Vector3>& ref_vectors,
    const std::vector<double>& weights = {}
) {
    size_t n = cam_rays.size();
    if (n < 2 || ref_vectors.size() != n) {
        return Quaternion::identity();
    }

    // Accumulate Davenport's attitude profile matrix B
    Matrix3 B = Matrix3::zeros();
    double total_w = 0.0;
    for (size_t i = 0; i < n; ++i) {
        double w = weights.empty() ? 1.0 : weights[i];
        total_w += w;
        const auto& wc = cam_rays[i];
        const auto& vi = ref_vectors[i];

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

    Matrix3 S = B + B.transpose();
    double sigma = B(0, 0) + B(1, 1) + B(2, 2);
    Vector3 Z(B(1, 2) - B(2, 1),
              B(2, 0) - B(0, 2),
              B(0, 1) - B(1, 0));

    // Form 4x4 Davenport K matrix
    double K[4][4];
    K[0][0] = sigma;
    K[0][1] = Z.x; K[0][2] = Z.y; K[0][3] = Z.z;
    K[1][0] = Z.x; K[2][0] = Z.y; K[3][0] = Z.z;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            K[i + 1][j + 1] = S(i, j) - (i == j ? sigma : 0.0);
        }
    }

    // 4x4 cyclic Jacobi eigensolver
    double V[4][4] = {
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };

    for (int sweep = 0; sweep < 30; ++sweep) {
        double off_diag = 0.0;
        for (int i = 0; i < 4; ++i) {
            for (int j = i + 1; j < 4; ++j) {
                off_diag += std::abs(K[i][j]);
            }
        }
        if (off_diag < 1e-15) break;

        for (int p = 0; p < 3; ++p) {
            for (int q = p + 1; q < 4; ++q) {
                double Apq = K[p][q];
                if (std::abs(Apq) < 1e-16) continue;

                double h = K[q][q] - K[p][p];
                double t;
                if (std::abs(h) > 1e-15) {
                    double tau = h / (2.0 * Apq);
                    t = (tau >= 0.0 ? 1.0 : -1.0) / (std::abs(tau) + std::sqrt(1.0 + tau * tau));
                } else {
                    t = (Apq >= 0.0 ? 1.0 : -1.0);
                }
                double c = 1.0 / std::sqrt(1.0 + t * t);
                double s = t * c;

                K[p][p] -= t * Apq;
                K[q][q] += t * Apq;
                K[p][q] = 0.0;
                K[q][p] = 0.0;

                for (int r = 0; r < 4; ++r) {
                    if (r != p && r != q) {
                        double Arp = K[r][p];
                        double Arq = K[r][q];
                        K[r][p] = c * Arp - s * Arq;
                        K[p][r] = K[r][p];
                        K[r][q] = s * Arp + c * Arq;
                        K[q][r] = K[r][q];
                    }
                }

                for (int r = 0; r < 4; ++r) {
                    double Vrp = V[r][p];
                    double Vrq = V[r][q];
                    V[r][p] = c * Vrp - s * Vrq;
                    V[r][q] = s * Vrp + c * Vrq;
                }
            }
        }
    }

    // Find eigenvector of maximum eigenvalue
    int max_idx = 0;
    double max_val = K[0][0];
    for (int i = 1; i < 4; ++i) {
        if (K[i][i] > max_val) {
            max_val = K[i][i];
            max_idx = i;
        }
    }

    // Hamilton quaternion mapping inertial to body: [w, -x, -y, -z]
    Quaternion q(V[0][max_idx], -V[1][max_idx], -V[2][max_idx], -V[3][max_idx]);
    if (q.w < 0.0) {
        q.w = -q.w; q.x = -q.x; q.y = -q.y; q.z = -q.z;
    }
    return q.normalized();
}

/**
 * @brief Solves Wahba's problem using Singular Value Decomposition (Markley, 1988).
 *
 * @details
 * Given B = sum w_i * b_i * r_i^T, computes SVD: B = U * Sigma * V^T.
 * The optimal attitude DCM is:
 *   A_opt = U * diag(1, 1, det(U)*det(V)) * V^T
 * and is converted directly to an attitude quaternion.
 *
 * SVD is computed by diagonalizing C = B^T * B using cyclic 3x3 Jacobi rotations.
 *
 * @reference
 * - Markley, F. L. (1988). "Attitude Determination Using Vector Observations and the
 *   Singular Value Decomposition." Journal of the Astronautical Sciences, 36(3), pp. 245–258.
 * - Delabie et al. (2013). "Highly Efficient Attitude-Estimation Algorithm for Star Trackers
 *   Using Optimal Image Matching." JGCD, 36(6), p. 1673.
 *
 * @param cam_rays Observed star unit vectors in camera frame.
 * @param ref_vectors Corresponding star unit vectors in celestial inertial frame.
 * @param weights Optional weights per star measurement.
 * @return Optimal attitude quaternion (inertial to camera).
 */
inline Quaternion solve_svd(
    const std::vector<Vector3>& cam_rays,
    const std::vector<Vector3>& ref_vectors,
    const std::vector<double>& weights = {}
) {
    size_t n = cam_rays.size();
    if (n < 2 || ref_vectors.size() != n) {
        return Quaternion::identity();
    }

    // Accumulate Davenport's attitude profile matrix B
    Matrix3 B = Matrix3::zeros();
    double total_w = 0.0;
    for (size_t i = 0; i < n; ++i) {
        double w = weights.empty() ? 1.0 : weights[i];
        total_w += w;
        const auto& wc = cam_rays[i];
        const auto& vi = ref_vectors[i];

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

    // Form symmetric 3x3 matrix C = B^T * B
    Matrix3 C = B.transpose() * B;

    // Diagonalize C using 3x3 cyclic Jacobi rotations: V^T C V = diag(lambda)
    Matrix3 V = Matrix3::identity();
    for (int sweep = 0; sweep < 30; ++sweep) {
        double off = std::abs(C(0, 1)) + std::abs(C(0, 2)) + std::abs(C(1, 2));
        if (off < 1e-15) break;

        for (int p = 0; p < 2; ++p) {
            for (int q = p + 1; q < 3; ++q) {
                double Apq = C(p, q);
                if (std::abs(Apq) < 1e-16) continue;

                double h = C(q, q) - C(p, p);
                double t;
                if (std::abs(h) > 1e-15) {
                    double tau = h / (2.0 * Apq);
                    t = (tau >= 0.0 ? 1.0 : -1.0) / (std::abs(tau) + std::sqrt(1.0 + tau * tau));
                } else {
                    t = (Apq >= 0.0 ? 1.0 : -1.0);
                }
                double c = 1.0 / std::sqrt(1.0 + t * t);
                double s = t * c;

                C(p, p) -= t * Apq;
                C(q, q) += t * Apq;
                C(p, q) = 0.0;
                C(q, p) = 0.0;

                for (int r = 0; r < 3; ++r) {
                    if (r != p && r != q) {
                        double Arp = C(r, p);
                        double Arq = C(r, q);
                        C(r, p) = c * Arp - s * Arq;
                        C(p, r) = C(r, p);
                        C(r, q) = s * Arp + c * Arq;
                        C(q, r) = C(r, q);
                    }
                }

                for (int r = 0; r < 3; ++r) {
                    double Vrp = V(r, p);
                    double Vrq = V(r, q);
                    V(r, p) = c * Vrp - s * Vrq;
                    V(r, q) = s * Vrp + c * Vrq;
                }
            }
        }
    }

    // Sort singular values in descending order
    int idx[3] = {0, 1, 2};
    if (C(idx[0], idx[0]) < C(idx[1], idx[1])) std::swap(idx[0], idx[1]);
    if (C(idx[1], idx[1]) < C(idx[2], idx[2])) std::swap(idx[1], idx[2]);
    if (C(idx[0], idx[0]) < C(idx[1], idx[1])) std::swap(idx[0], idx[1]);

    Matrix3 V_sorted;
    double sig[3];
    for (int j = 0; j < 3; ++j) {
        sig[j] = std::sqrt(std::max(0.0, C(idx[j], idx[j])));
        for (int r = 0; r < 3; ++r) {
            V_sorted(r, j) = V(r, idx[j]);
        }
    }

    // Compute left singular vectors: u_j = B * v_j / sigma_j
    Matrix3 U;
    for (int j = 0; j < 2; ++j) {
        Vector3 vj(V_sorted(0, j), V_sorted(1, j), V_sorted(2, j));
        Vector3 uj = B * vj;
        if (sig[j] > 1e-9) {
            uj = uj / sig[j];
        } else {
            uj = Vector3(j == 0 ? 1.0 : 0.0, j == 1 ? 1.0 : 0.0, 0.0);
        }
        U(0, j) = uj.x; U(1, j) = uj.y; U(2, j) = uj.z;
    }
    Vector3 u0(U(0, 0), U(1, 0), U(2, 0));
    Vector3 u1(U(0, 1), U(1, 1), U(2, 1));
    Vector3 u2 = u0.cross(u1).normalized();
    U(0, 2) = u2.x; U(1, 2) = u2.y; U(2, 2) = u2.z;

    double detU = U.determinant();
    double detV = V_sorted.determinant();
    double d = (detU * detV >= 0.0) ? 1.0 : -1.0;

    // A_opt = U * diag(1, 1, d) * V^T
    Matrix3 diag(1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, d);
    Matrix3 A = U * diag * V_sorted.transpose();

    return Quaternion::from_matrix(A);
}

} // namespace star_tracker
