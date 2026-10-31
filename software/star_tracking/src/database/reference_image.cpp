/**
 * @file reference_image.cpp
 * @brief Implementation of Delabie's 3-star centroid method and invariant feature extraction.
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." Journal of Guidance, Control, and Dynamics, 36(2),
 *   pp. 476–488. Section II.C.2 "Centroid Method" and Section II.E.1. DOI: 10.2514/1.56860.
 */

#include "database/reference_image.hpp"
#include <algorithm>
#include <cmath>

namespace star_tracker {

CentroidFeatures CentroidFeatures::compute(const std::vector<ImageStar>& triplet) {
    CentroidFeatures feat;
    if (triplet.size() < 3) {
        feat.valid = false;
        return feat;
    }

    const auto& s1 = triplet[0];
    const auto& s2 = triplet[1];
    const auto& s3 = triplet[2];

    // Step 1: Geometric centroid calculation: C = (S1 + S2 + S3) / 3
    // (Delabie et al., 2013, Section II.C.2)
    double cx = (s1.x + s2.x + s3.x) / 3.0;
    double cy = (s1.y + s2.y + s3.y) / 3.0;
    feat.centroid_x = cx;
    feat.centroid_y = cy;

    // Step 2: Radial displacement vectors from centroid to each star
    double u1x = s1.x - cx, u1y = s1.y - cy;
    double u2x = s2.x - cx, u2y = s2.y - cy;
    double u3x = s3.x - cx, u3y = s3.y - cy;

    double d1 = std::sqrt(u1x * u1x + u1y * u1y);
    double d2 = std::sqrt(u2x * u2x + u2y * u2y);
    double d3 = std::sqrt(u3x * u3x + u3y * u3y);

    // Reject degenerate triplets with coincident stars
    if (d1 < 1e-4 || d2 < 1e-4 || d3 < 1e-4) {
        feat.valid = false;
        return feat;
    }

    // Lambda to compute subtended angle between two 2D radial vectors
    auto calc_angle = [](double ax, double ay, double da, double bx, double by, double db) {
        double dot = (ax * bx + ay * by) / (da * db);
        return std::acos(std::clamp(dot, -1.0, 1.0));
    };

    double a12 = calc_angle(u1x, u1y, d1, u2x, u2y, d2);
    double a23 = calc_angle(u2x, u2y, d2, u3x, u3y, d3);
    double a31 = calc_angle(u3x, u3y, d3, u1x, u1y, d1);

    // Step 3: Identify the star pair that subtends the minimal angle alpha_min
    // (Delabie et al., 2013, Section II.C.2, p. 478)
    const ImageStar* starA = nullptr;
    const ImageStar* starB = nullptr;
    const ImageStar* starC = nullptr;
    double min_ang = a12;

    if (a12 <= a23 && a12 <= a31) {
        starA = &s1; starB = &s2; starC = &s3;
        min_ang = a12;
    } else if (a23 <= a12 && a23 <= a31) {
        starA = &s2; starB = &s3; starC = &s1;
        min_ang = a23;
    } else {
        starA = &s3; starB = &s1; starC = &s2;
        min_ang = a31;
    }

    double uAx = starA->x - cx, uAy = starA->y - cy;
    double uBx = starB->x - cx, uBy = starB->y - cy;
    double uCx = starC->x - cx, uCy = starC->y - cy;

    double dA = std::sqrt(uAx * uAx + uAy * uAy);
    double dB = std::sqrt(uBx * uBx + uBy * uBy);
    double dC = std::sqrt(uCx * uCx + uCy * uCy);

    // Step 4: Enforce canonical distance ordering dA <= dB on the minimal-angle pair
    // This removes the 180° orientation ambiguity between star A and star B
    if (dA > dB) {
        std::swap(starA, starB);
        std::swap(uAx, uBx);
        std::swap(uAy, uBy);
        std::swap(dA, dB);
    }

    // Step 5: Store 6 invariant geometric features: 3 distances and 3 angles
    feat.distances[0] = dA;
    feat.distances[1] = dB;
    feat.distances[2] = dC;

    feat.angles[0] = min_ang;
    feat.angles[1] = calc_angle(uAx, uAy, dA, uCx, uCy, dC);
    feat.angles[2] = calc_angle(uBx, uBy, dB, uCx, uCy, dC);

    // Step 6: Orientation angle psi defined along the bisector of (uA, uB)
    double ox = uAx / dA + uBx / dB;
    double oy = uAy / dA + uBy / dB;
    if (ox * ox + oy * oy < 1e-6) {
        ox = uAx;
        oy = uAy;
    }
    feat.orientation_angle = std::atan2(oy, ox);

    feat.star_ids[0] = starA->catalog_id;
    feat.star_ids[1] = starB->catalog_id;
    feat.star_ids[2] = starC->catalog_id;

    feat.valid = true;
    return feat;
}

} // namespace star_tracker
