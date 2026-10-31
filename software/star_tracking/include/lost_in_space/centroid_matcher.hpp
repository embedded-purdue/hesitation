/**
 * @file centroid_matcher.hpp
 * @brief 2D focal-plane geometric pattern alignment using Delabie's centroid method.
 *
 * @details
 * In Delabie et al. (2013, "SDT", Section II.C "Matching the Camera Image with the Database Images"),
 * two images must be aligned in 2D translation and roll rotation so that their star patterns
 * overlap before querying the distance transform map:
 *   x_c = x_r * cos(phi) - y_r * sin(phi) + t_x
 *   y_c = x_r * sin(phi) + y_r * cos(phi) + t_y
 *
 * Method Derivation:
 * 1. Rotation Alignment:
 *    The roll angle phi is computed directly from the difference between the canonical orientation
 *    angles of the matched star triplets:
 *      phi = psi_cam - psi_ref
 *    normalized to [-pi, pi].
 * 2. Translation Alignment:
 *    With rotation resolved, the 2D translation (t_x, t_y) maps the reference centroid (rx, ry)
 *    directly onto the camera centroid (cx, cy):
 *      t_x = cx - (rx * cos(phi) - ry * sin(phi))
 *      t_y = cy - (rx * sin(phi) + ry * cos(phi))
 *
 * Invariance & Speed:
 * As shown in Delabie et al. Table 1, the centroid method reduces required search combinations
 * by 3x to 5x compared to the two-brightest-stars permutation method, while maintaining extreme
 * robustness to magnitude measurement noise and star order inversions.
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." Journal of Guidance, Control, and Dynamics, 36(2),
 *   pp. 476–488. Section II.C.2 "Centroid Method", Figs. 5–6, Table 1. DOI: 10.2514/1.56860.
 */

#pragma once

#include "database/reference_image.hpp"
#include <cmath>

namespace star_tracker {

/**
 * @struct Alignment
 * @brief 2D Euclidean rigid-body transformation (translation and roll rotation) on the focal plane.
 */
struct Alignment {
    double translation_x{0.0}; ///< Translation along focal-plane X axis in pixels (t_x)
    double translation_y{0.0}; ///< Translation along focal-plane Y axis in pixels (t_y)
    double rotation{0.0};      ///< In-plane roll rotation angle phi in radians

    bool valid{false};         ///< True if transformation is mathematically well-defined

    /**
     * @brief Transforms a point from reference image coordinates (xr, yr) into camera coordinates (xc, yc).
     * @details
     * xc = xr * cos(phi) - yr * sin(phi) + tx
     * yc = xr * sin(phi) + yr * cos(phi) + ty
     *
     * @param xr Reference point X.
     * @param yr Reference point Y.
     * @param[out] xc Transformed camera X coordinate.
     * @param[out] yc Transformed camera Y coordinate.
     */
    void transform_point(double xr, double yr, double& xc, double& yc) const {
        double c = std::cos(rotation);
        double s = std::sin(rotation);
        xc = xr * c - yr * s + translation_x;
        yc = xr * s + yr * c + translation_y;
    }
};

/**
 * @class CentroidMatcher
 * @brief Solves translation and roll rotation alignment between camera and database star triplets.
 */
class CentroidMatcher {
public:
    CentroidMatcher() = default;

    /**
     * @brief Computes 2D rigid transformation mapping reference features to camera features.
     * @details
     * Evaluates roll difference phi = psi_cam - psi_ref and shifts centroid coordinates.
     *
     * @param cam_features Triplet centroid features extracted from camera image.
     * @param ref_features Precomputed triplet centroid features from database image.
     * @return Alignment struct with translation and rotation parameters.
     */
    static Alignment compute_alignment(
        const CentroidFeatures& cam_features,
        const CentroidFeatures& ref_features
    );

    /**
     * @brief Searches combinations among the brightest camera stars to find an alignment matching reference.
     * @param camera_stars Detected camera stars sorted by brightness.
     * @param reference Target database reference image.
     * @return Alignment if a valid triplet match is found, invalid Alignment otherwise.
     */
    Alignment align(
        const std::vector<ImageStar>& camera_stars,
        const ReferenceImage& reference
    );
};

} // namespace star_tracker
