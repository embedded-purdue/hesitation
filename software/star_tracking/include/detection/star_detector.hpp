/**
 * @file star_detector.hpp
 * @brief Autonomous star detection using connected component labeling and adaptive thresholding.
 *
 * @details
 * This module identifies candidate star spots in camera images.
 *
 * Processing Stages:
 * 1. Background Estimation & Adaptive Thresholding:
 *    Computes image mean mu and standard deviation sigma across all sensor pixels:
 *      Threshold = mu + k_sigma * sigma
 *    Adapts automatically to variations in sensor dark current, thermal drift, and stray light.
 * 2. 8-Connected Component Labeling (CCL):
 *    Employs a queue-based Breadth-First Search (BFS) over 8-connected neighborhoods to segment
 *    pixels above threshold into discrete spatial clusters.
 * 3. Morphological Cluster Filtering:
 *    Discards clusters that fail physical validity criteria:
 *    - min_region_size (e.g., < 2 pixels): Rejects single hot pixels and cosmic ray strikes.
 *    - max_region_size (e.g., > 200 pixels): Rejects extended illuminated bodies (Moon, Earth limb,
 *      sunlight glints, or bright orbital debris).
 *    - min_total_flux: Rejects faint noise fluctuations below signal-to-noise limit.
 * 4. Subpixel Centroiding:
 *    Applies `CentroidExtractor` to determine subpixel center coordinates and integrated flux.
 * 5. Photometric Sorting:
 *    Sorts all detected stars in descending order of brightness, ensuring the brightest stars
 *    appear first for Delabie's triplet centroid matching (Delabie et al., 2013, "SDT").
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." JGCD, 36(2), pp. 476–488. Section II.A
 *   "Distance Transform" and Section II.C "Matching". DOI: 10.2514/1.56860.
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." JGCD, 36(6), pp. 1672–1680.
 *   DOI: 10.2514/1.61082.
 */

#pragma once

#include "camera/camera_model.hpp"
#include "catalog/star.hpp"
#include "detection/centroid_extractor.hpp"
#include "detection/image.hpp"
#include <vector>

namespace star_tracker {

/**
 * @struct StarDetectorConfig
 * @brief Configuration parameters for the star detection and segmentation engine.
 */
struct StarDetectorConfig {
    double threshold{25.0};        ///< Fixed brightness threshold (used when adaptive_threshold is false)
    int min_region_size{2};        ///< Minimum connected pixel count to qualify as star spot (rejects hot pixels)
    int max_region_size{200};      ///< Maximum connected pixel count (rejects planets/Moon/debris blooms)
    double min_total_flux{30.0};   ///< Minimum integrated net brightness above background floor
    bool adaptive_threshold{true}; ///< Compute dynamic threshold from image statistics (mu + k*sigma)
    double noise_sigma_mult{4.0};  ///< Multiplier k for background noise standard deviation (default: 4-sigma)
};

/**
 * @class StarDetector
 * @brief Pipeline stage that extracts a sorted list of subpixel star spots from raw camera images.
 */
class StarDetector {
public:
    /**
     * @brief Constructor with optional configuration.
     * @param config Detector tuning configuration.
     */
    explicit StarDetector(const StarDetectorConfig& config = StarDetectorConfig())
        : config_(config) {}

    /**
     * @brief Executes star detection pipeline on an incoming image frame.
     * @details
     * Applies adaptive thresholding, 8-connectivity clustering, morphological region filtering,
     * subpixel centroid extraction, and brightness sorting.
     *
     * @param image Input grayscale image buffer.
     * @param camera Camera geometric bounds and intrinsic model.
     * @return List of detected ImageStar objects sorted in descending order of brightness.
     */
    std::vector<ImageStar> detect_stars(
        const Image& image,
        const CameraModel& camera
    ) const;

    /// Read-only access to detector configuration.
    const StarDetectorConfig& config() const { return config_; }

    /// Updates detector configuration.
    void set_config(const StarDetectorConfig& config) { config_ = config; }

private:
    StarDetectorConfig config_;
};

} // namespace star_tracker
