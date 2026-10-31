/**
 * @file database_generator.hpp
 * @brief Offline generation of uniform celestial reference databases via Fibonacci sphere covering.
 *
 * @details
 * In Delabie et al. (2013, "SDT", Section II.B "Database Images"), the lost-in-space algorithm
 * searches a database of preprocessed reference images. To achieve uniform spherical sky coverage
 * without clustering at the poles, centers are distributed evenly over the celestial sphere.
 *
 * Fibonacci Sphere Spiral Distribution:
 * Points are spaced along a spherical spiral using the golden ratio phi_golden = pi * (3 - sqrt(5)):
 *   y_i = 1 - (2 * i) / (N - 1)  for i in [0, N-1]
 *   r_i = sqrt( max(0, 1 - y_i^2) )
 *   theta_i = i * phi_golden
 *   x_i = r_i * cos(theta_i), z_i = r_i * sin(theta_i)
 *
 * Optical Axis Alignment:
 * For each center vector u_i = [x_i, y_i, z_i]^T, a rotation quaternion q is constructed such that:
 *   R(q) * u_i = [0, 0, 1]^T  (+Z optical boresight)
 *
 * Feature Precomputation:
 * For each reference pointing direction:
 * 1. Stars brighter than magnitude threshold (e.g., 5.5) are projected onto the focal plane.
 * 2. All combinations of 3 stars from the top n brightest stars (typically n = 5, yielding
 *    n_comb = 10 combinations) are evaluated to precompute Delabie's 6 invariant centroid features.
 * (Delabie et al., 2013, Section II.E.2 "Preprocessed Database").
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." JGCD, 36(2), pp. 476–488. Section II.B
 *   "Database Images" and Section II.E.2. DOI: 10.2514/1.56860.
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." JGCD, 36(6), pp. 1672–1680.
 *   DOI: 10.2514/1.61082.
 */

#pragma once

#include "camera/camera_model.hpp"
#include "catalog/star_catalog.hpp"
#include "database/reference_database.hpp"

namespace star_tracker {

/**
 * @struct DatabaseGeneratorConfig
 * @brief Configuration parameters for reference image sphere generation.
 */
struct DatabaseGeneratorConfig {
    uint32_t num_images{300};         ///< Number of discrete reference images uniformly covering the celestial sphere
    double magnitude_threshold{5.5};  ///< Maximum visual magnitude cutoff (e.g. 5.5 ensures >=3 stars per FOV)
    double margin_pixels{5.0};        ///< Sensor border guard margin in pixels
    std::string catalog_source{"Hipparcos"}; ///< Stellar catalog label
};

/**
 * @class DatabaseGenerator
 * @brief Engine for offline construction and feature precomputation of the reference database.
 */
class DatabaseGenerator {
public:
    /**
     * @brief Constructor with optional generation configuration.
     * @param config Generation settings.
     */
    explicit DatabaseGenerator(const DatabaseGeneratorConfig& config = DatabaseGeneratorConfig())
        : config_(config) {}

    /**
     * @brief Generates a complete reference database from an astronomical star catalog.
     * @details
     * Synthesizes N reference pointings distributed via Fibonacci spherical covering, projects
     * visible stars, and precomputes all Delabie centroid features.
     *
     * @param catalog Astronomical star catalog.
     * @param camera Target camera intrinsic model and field of view.
     * @return Generated ReferenceDatabase populated with reference images.
     */
    ReferenceDatabase generate(
        const StarCatalog& catalog,
        const CameraModel& camera
    ) const;

    /// Read-only access to generator configuration.
    const DatabaseGeneratorConfig& config() const { return config_; }

    /// Updates generator configuration.
    void set_config(const DatabaseGeneratorConfig& config) { config_ = config; }

private:
    DatabaseGeneratorConfig config_;
};

} // namespace star_tracker
