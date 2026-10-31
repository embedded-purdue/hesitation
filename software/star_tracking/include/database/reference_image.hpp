/**
 * @file reference_image.hpp
 * @brief Geometric star triplet centroid features and reference image structures.
 *
 * @details
 * This file implements the geometric centroid feature structures defined by Delabie et al.
 * (2013, "SDT", Section II.C.2 "Centroid Method" and Section II.E.1 "Discarding False Images").
 *
 * Delabie Centroid Method Theory:
 * Traditional star trackers pair stars by matching triangles via permutations of the 2 brightest stars,
 * requiring n! / (n - 2)! permutations (e.g., 20 to 42 iterations for 5 to 7 stars).
 * Delabie et al. introduced the Centroid Method, which uses triplets of bright stars:
 * 1. Centroid Calculation:
 *    C = (S_1 + S_2 + S_3) / 3
 *    The geometric center C is invariant to the relative brightness order of the 3 stars.
 * 2. Radial Vectors & Distances:
 *    u_i = S_i - C,  d_i = ||u_i||  for i in {1, 2, 3}
 * 3. Pairwise Subtended Angles:
 *    alpha_{ij} = arccos( (u_i · u_j) / (d_i * d_j) )
 * 4. Canonical Ordering via Smallest Angle:
 *    Identifies the pair (star A, star B) subtending the minimal angle alpha_min.
 *    Enforces d_A <= d_B to resolve the 180° ambiguity. Star C is the third star.
 * 5. 6 Invariant Geometric Features:
 *    - Three centroid distances: (d_A, d_B, d_C)
 *    - Three pairwise angles: (alpha_min, alpha_AC, alpha_BC)
 *    These 6 invariant features are used by `CandidateFilter` to discard >90% of database
 *    images prior to querying the computationally heavier Distance Transform map.
 * 6. Orientation Angle psi:
 *    Computed from the bisector of u_A and u_B, resolving image roll around the optical axis.
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." JGCD, 36(2), pp. 476–488. Section II.C.2
 *   "Centroid Method", Section II.E.1 "Discarding False Images", and Table 1. DOI: 10.2514/1.56860.
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." JGCD, 36(6), pp. 1672–1680.
 *   DOI: 10.2514/1.61082.
 */

#pragma once

#include "catalog/star.hpp"
#include "math/quaternion.hpp"
#include <array>
#include <cstdint>
#include <vector>

namespace star_tracker {

/**
 * @struct CentroidFeatures
 * @brief 6 invariant geometric features extracted from a star triplet using Delabie's centroid method.
 */
struct CentroidFeatures {
    bool valid{false}; ///< True if feature computation succeeded on non-degenerate triplet

    // Focal-plane coordinates of the 3-star geometric centroid: C = (S1 + S2 + S3) / 3
    double centroid_x{0.0}; ///< Centroid X coordinate in pixels
    double centroid_y{0.0}; ///< Centroid Y coordinate in pixels

    /**
     * @brief Distances from centroid to the 3 stars: [d_A, d_B, d_C] (pixels).
     * Ordered such that stars A and B form the smallest pairwise angle alpha_min, and d_A <= d_B.
     */
    std::array<double, 3> distances{0.0, 0.0, 0.0};

    /**
     * @brief Pairwise subtended angles between the 3 radial vectors (radians).
     * angles[0] = alpha_min (between star A and star B)
     * angles[1] = alpha_AC (between star A and star C)
     * angles[2] = alpha_BC (between star B and star C)
     */
    std::array<double, 3> angles{0.0, 0.0, 0.0};

    /**
     * @brief Orientation angle psi (radians) of the star pair (A, B) relative to centroid in image plane.
     * Resolves the roll rotation angle between camera frame and database reference frame.
     */
    double orientation_angle{0.0};

    /// Catalog IDs or indices of stars [A, B, C]
    std::array<uint64_t, 3> star_ids{0, 0, 0};

    /**
     * @brief Computes Delabie centroid features from an arbitrary triplet of 3 image stars.
     * @details
     * Implements canonical ordering based on the minimum pairwise angle alpha_min and d_A <= d_B.
     * @param triplet Vector containing exactly 3 ImageStar objects.
     * @return CentroidFeatures object (valid = true if successful, false if degenerate).
     */
    static CentroidFeatures compute(const std::vector<ImageStar>& triplet);
};

/**
 * @struct ReferenceImage
 * @brief Precomputed synthetic database image representing a discrete pointing direction on the sky.
 */
struct ReferenceImage {
    uint64_t id{0}; ///< Unique database image index

    /// True attitude quaternion (celestial inertial to camera sensor frame) of this reference direction
    Quaternion attitude;

    /// Projected catalog stars in focal-plane pixel coordinates, sorted by brightness
    std::vector<ImageStar> stars;

    /**
     * @brief Precomputed geometric centroid features for all combinations of the brightest stars.
     * Stored in the preprocessed database (Delabie et al., 2013, Section II.E.2).
     */
    std::vector<CentroidFeatures> feature_combinations;

    /// Primary centroid feature constructed from the top 3 brightest stars
    CentroidFeatures centroid_features;
};

} // namespace star_tracker
