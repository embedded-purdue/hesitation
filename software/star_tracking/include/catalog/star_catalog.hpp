/**
 * @file star_catalog.hpp
 * @brief Astronomical star catalog management, CSV loader, and focal-plane projection.
 *
 * @details
 * This class manages an on-board or ground database of reference stars (e.g., from the Hipparcos
 * catalog).
 *
 * Operational Responsibilities:
 * 1. CSV Catalog Ingestion: Loads astronomical records (ID, Right Ascension, Declination,
 *    Visual Magnitude) from disk.
 * 2. Magnitude Filtering: Reduces catalog size to stars brighter than a selected threshold
 *    (e.g., visual magnitude <= 5.5), ensuring bounded memory footprint and real-time execution.
 * 3. Perspective Projection: Given any spacecraft attitude quaternion q (inertial to camera frame)
 *    and camera intrinsic model, projects all stars lying within the camera's instantaneous
 *    field-of-view (FOV) onto the 2D focal plane.
 *    Stars are transformed via:
 *      v_cam = q * v_inertial * q^-1
 *    and projected via the pinhole camera model.
 *    The resulting list of `ImageStar`s is automatically sorted by brightness descending.
 *
 * In Delabie et al. (2013, "SDT"), database construction uses covering spheres and projection of
 * Hipparcos stars to generate reference images. In Delabie et al. (2013, "AIM"), projecting visible
 * catalog stars around a coarse attitude estimate q_dat generates the reference image used for
 * sub-pixel image matching.
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." JGCD, 36(2), pp. 476–488. DOI: 10.2514/1.56860.
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." JGCD, 36(6), pp. 1672–1680.
 *   DOI: 10.2514/1.61082.
 */

#pragma once

#include "camera/camera_model.hpp"
#include "catalog/star.hpp"
#include "math/quaternion.hpp"
#include "math/vector.hpp"
#include <string>
#include <vector>

namespace star_tracker {

/**
 * @class StarCatalog
 * @brief Container and query engine for celestial reference stars.
 */
class StarCatalog {
public:
    /// Default constructor.
    StarCatalog() = default;

    /**
     * @brief Loads catalog stars from a CSV file.
     * @details
     * Expected CSV columns: id, right_ascension (rad), declination (rad), visual_magnitude.
     * Handles comments (#) and header rows gracefully.
     * @param filepath Path to the CSV file.
     * @return true if successfully parsed with at least one star, false on I/O or format error.
     */
    bool load_from_csv(const std::string& filepath);

    /**
     * @brief Appends a single star to the catalog.
     * @param star Astronomical star record.
     */
    void add_star(const Star& star) {
        stars_.push_back(star);
    }

    /// Read-only access to the list of catalog stars.
    const std::vector<Star>& stars() const { return stars_; }

    /// Returns the number of stars currently loaded in the catalog.
    size_t size() const { return stars_.size(); }

    /// Checks if the catalog is empty.
    bool empty() const { return stars_.empty(); }

    /**
     * @brief Creates a filtered sub-catalog containing only stars brighter than or equal to max_mag.
     * @details
     * In Pogson's scale, smaller magnitude values represent brighter stars (e.g. mag <= 5.5).
     * @param max_mag Maximum apparent visual magnitude threshold.
     * @return Filtered StarCatalog.
     */
    StarCatalog filter_by_magnitude(double max_mag) const;

    /**
     * @brief Projects all catalog stars visible in the camera field of view for a given attitude.
     * @details
     * Transforms each star's inertial unit vector into the camera frame using attitude quaternion q:
     *   v_cam = R(attitude) * v_inertial
     * Stars with v_cam.z > 0 are projected onto the focal plane via the camera model.
     * Returns detected focal-plane stars sorted in descending order of brightness (ascending magnitude).
     *
     * @param attitude Spacecraft attitude quaternion mapping celestial inertial frame to camera frame.
     * @param camera Pinhole camera intrinsic calibration and distortion model.
     * @param max_mag Faintest magnitude to include.
     * @param margin_pixels Guard margin in pixels from the sensor border.
     * @return Vector of projected ImageStar objects within the field-of-view, sorted by brightness.
     */
    std::vector<ImageStar> project_visible_stars(
        const Quaternion& attitude,
        const CameraModel& camera,
        double max_mag = 100.0,
        double margin_pixels = 0.0
    ) const;

    /**
     * @brief Searches for a catalog star by its unique ID.
     * @param id Unique star identifier (e.g., Hipparcos HIP number).
     * @return Pointer to Star if found, nullptr otherwise.
     */
    const Star* find_star(uint64_t id) const;

private:
    std::vector<Star> stars_; ///< Internal storage of catalog stars
};

} // namespace star_tracker
