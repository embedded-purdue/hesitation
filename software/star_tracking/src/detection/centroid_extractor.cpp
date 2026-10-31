/**
 * @file centroid_extractor.cpp
 * @brief Implementation of subpixel Center-of-Gravity centroid extraction and flux integration.
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." Journal of Guidance, Control, and Dynamics, 36(2),
 *   pp. 476–488. DOI: 10.2514/1.56860.
 */

#include "detection/centroid_extractor.hpp"
#include <algorithm>
#include <cmath>

namespace star_tracker {

ImageStar CentroidExtractor::extract_centroid(
    const std::vector<PixelCoord>& region,
    double background_threshold
) {
    if (region.empty()) {
        return ImageStar(0, 0.0, 0.0, 0.0, 0.0);
    }

    double sum_i = 0.0;
    double sum_ix = 0.0;
    double sum_iy = 0.0;

    // Accumulate zeroth and first image intensity moments above background floor
    for (const auto& p : region) {
        double weight = std::max(0.0, p.intensity - background_threshold);
        sum_i += weight;
        sum_ix += weight * p.x;
        sum_iy += weight * p.y;
    }

    // Guard against faint blobs or all pixels below background threshold
    if (sum_i <= 1e-9) {
        // Fallback: unweighted geometric average of pixel coordinates
        double avg_x = 0.0, avg_y = 0.0;
        for (const auto& p : region) {
            avg_x += p.x;
            avg_y += p.y;
        }
        return ImageStar(0, avg_x / region.size(), avg_y / region.size(), 0.0, 0.0);
    }

    // Center-of-Gravity (CoG): 1st moment divided by 0th moment
    double xc = sum_ix / sum_i;
    double yc = sum_iy / sum_i;

    // Photometric apparent magnitude conversion:
    // m = -2.5 * log10(Flux) + ZeroPoint
    // ZeroPoint = 15.0 calibrates typical digital counts to visual stellar magnitude scale
    double mag = -2.5 * std::log10(std::max(1.0, sum_i)) + 15.0;

    return ImageStar(0, xc, yc, sum_i, mag);
}

} // namespace star_tracker
