/**
 * @file centroid_extractor.hpp
 * @brief Subpixel Center-of-Gravity (CoG) centroid extraction and stellar photometry.
 *
 * @details
 * In star trackers, the optical system is deliberately slightly defocused so that each star's
 * Point Spread Function (PSF) illuminates a 3x3 to 5x5 pixel cluster on the detector. This
 * enables subpixel interpolation beyond the Nyquist-Shannon sampling limit of individual pixels.
 *
 * Centroid Extraction Formula:
 * For a connected pixel region Omega above the background threshold I_bg:
 *   w_i = max(0, I_i - I_bg)
 *   x_c = sum_{i in Omega} (w_i * x_i) / sum_{i in Omega} w_i
 *   y_c = sum_{i in Omega} (w_i * y_i) / sum_{i in Omega} w_i
 *
 * Total Integrated Flux:
 *   Flux = sum_{i in Omega} w_i
 *
 * Apparent Magnitude Estimation:
 * From Pogson's relationship:
 *   mag = -2.5 * log10( max(1.0, Flux) ) + C_cal
 * where C_cal is the photometric zero-point calibration constant.
 *
 * Accuracy Context:
 * As documented in Delabie et al. (2013, "SDT", Section III.A), centroiding algorithms on CMOS
 * star trackers achieve nominal 1-sigma positional errors of 0.05 to 0.1 pixels (~2 to 4 arcsec
 * for standard optics). Delabie's SDT algorithm is tested and verified to maintain >99%
 * identification accuracy even when centroid positional errors reach 1000 arcseconds (>7 pixels).
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." JGCD, 36(2), pp. 476–488. Section III.A
 *   "Positional Error". DOI: 10.2514/1.56860.
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." JGCD, 36(6), pp. 1672–1680.
 *   DOI: 10.2514/1.61082.
 */

#pragma once

#include "catalog/star.hpp"
#include <vector>

namespace star_tracker {

/**
 * @struct PixelCoord
 * @brief Discrete sensor pixel coordinate and measured raw intensity.
 */
struct PixelCoord {
    int x{0};              ///< Horizontal pixel coordinate on sensor grid [0, width-1]
    int y{0};              ///< Vertical pixel coordinate on sensor grid [0, height-1]
    double intensity{0.0}; ///< Measured digital number (DN) / pixel count
};

/**
 * @class CentroidExtractor
 * @brief Computes subpixel star centroids and photometric properties from pixel clusters.
 */
class CentroidExtractor {
public:
    CentroidExtractor() = default;

    /**
     * @brief Extracts subpixel centroid (x_c, y_c) using brightness-weighted center-of-gravity (CoG).
     * @details
     * Subtracts the background threshold from each pixel intensity before calculating moments.
     * If the integrated net brightness is zero or negative, defaults to unweighted geometric center.
     *
     * @param region List of connected pixels composing the star spot.
     * @param background_threshold Estimated local or global background noise floor.
     * @return ImageStar containing subpixel coordinates, total flux, and estimated magnitude.
     */
    static ImageStar extract_centroid(
        const std::vector<PixelCoord>& region,
        double background_threshold = 0.0
    );
};

} // namespace star_tracker
