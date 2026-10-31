/**
 * @file reference_database.hpp
 * @brief Binary reference database serialization, metadata validation, and fast runtime lookup.
 *
 * @details
 * In the Delabie Shortest Distance Transform (SDT) architecture, comparing incoming camera images
 * directly to thousands of catalog stars on every frame would be computationally prohibitive.
 * Instead, the sky is discretized into precomputed reference images evenly distributed on a
 * sphere (Delabie et al., 2013, "SDT", Section II.B & II.E.2).
 *
 * This class provides:
 * 1. Compact Binary Serialization (`STRKDB01`):
 *    Stores the full database (reference attitudes, visible projected stars, and precomputed
 *    triplet centroid combinations) in an efficient binary representation for fast cold-boot
 *    loading on spaceflight avionics.
 * 2. Hardware Compatibility Verification:
 *    Ensures that the database loaded into memory strictly matches the physical camera parameters
 *    (resolution, focal length, field-of-view, lens distortion, and magnitude limits) within
 *    numerical tolerance before tracking operations commence.
 * 3. JSON Metadata Export:
 *    Generates human- and machine-readable metadata summaries for configuration audits and
 *    Hardware-In-The-Loop (HITL) integration.
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." JGCD, 36(2), pp. 476–488. Section II.B
 *   "Database Images" and Section II.E.2 "Preprocessed Database". DOI: 10.2514/1.56860.
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." JGCD, 36(6), pp. 1672–1680.
 *   DOI: 10.2514/1.61082.
 */

#pragma once

#include "camera/camera_model.hpp"
#include "database/reference_image.hpp"
#include <string>
#include <vector>

namespace star_tracker {

/**
 * @struct DatabaseMetadata
 * @brief Camera intrinsic and catalog parameters associated with a generated reference database.
 */
struct DatabaseMetadata {
    uint32_t format_version{1};        ///< Database format version (STRKDB01)
    std::string catalog_source{"Hipparcos"}; ///< Source stellar catalog (e.g. Hipparcos, Tycho-2)
    int camera_width{1024};             ///< Sensor horizontal resolution in pixels
    int camera_height{1024};            ///< Sensor vertical resolution in pixels
    double horizontal_fov{0.34906585};  ///< Horizontal FOV in radians (20 degrees)
    double vertical_fov{0.34906585};    ///< Vertical FOV in radians (20 degrees)
    double focal_length{1000.0};        ///< Camera focal length in pixels
    double distortion_k1{0.0};          ///< 3rd-order radial distortion coefficient
    double distortion_k2{0.0};          ///< 5th-order radial distortion coefficient
    double magnitude_threshold{5.5};    ///< Apparent magnitude cutoff used during generation
    uint32_t num_images{0};             ///< Total count of reference pointing images
};

/**
 * @class ReferenceDatabase
 * @brief Repository of precomputed reference star images and geometric features covering the sky.
 */
class ReferenceDatabase {
public:
    ReferenceDatabase() = default;

    /**
     * @brief Serializes the database to a compact binary file on disk.
     * @param filepath Destination binary file path.
     * @param meta Associated database configuration metadata.
     * @return true if successfully written, false on I/O error.
     */
    bool save_binary(const std::string& filepath, const DatabaseMetadata& meta) const;

    /**
     * @brief Deserializes the database from a compact binary file.
     * @param filepath Source binary file path.
     * @param[out] meta Optional pointer to receive deserialized metadata.
     * @return true if successfully loaded, false on file not found, bad magic, or format mismatch.
     */
    bool load_binary(const std::string& filepath, DatabaseMetadata* meta = nullptr);

    /**
     * @brief Exports database metadata to a JSON file.
     * @param filepath Output JSON path.
     * @param meta Metadata to export.
     * @return true if successful, false on I/O error.
     */
    bool save_metadata_json(const std::string& filepath, const DatabaseMetadata& meta) const;

    /**
     * @brief Ingests database metadata from a JSON file.
     * @param filepath Source JSON path.
     * @param[out] meta Struct to populate.
     * @return true if successful, false on error.
     */
    bool load_metadata_json(const std::string& filepath, DatabaseMetadata& meta) const;

    /**
     * @brief Verifies optical compatibility between runtime camera and database parameters.
     * @param camera Active runtime camera model.
     * @param meta Loaded database metadata.
     * @param tolerance Relative error tolerance for continuous parameters (default: 1e-3).
     * @return true if fully compatible, false otherwise.
     */
    bool is_compatible(const CameraModel& camera, const DatabaseMetadata& meta, double tolerance = 1e-3) const;

    /// Appends a new precomputed reference image to the database.
    void add_image(const ReferenceImage& img) { images_.push_back(img); }

    /// Read-only access to all reference images.
    const std::vector<ReferenceImage>& images() const { return images_; }

    /// Total count of reference images in the database.
    size_t size() const { return images_.size(); }

    /// Checks if database contains zero reference images.
    bool empty() const { return images_.empty(); }

    /**
     * @brief Finds a reference image by its ID.
     * @param id Reference image identifier.
     * @return Pointer to ReferenceImage if found, nullptr otherwise.
     */
    const ReferenceImage* find_image(uint64_t id) const;

    /// Read-only access to stored database metadata.
    const DatabaseMetadata& metadata() const { return metadata_; }

private:
    std::vector<ReferenceImage> images_;
    DatabaseMetadata metadata_;
};

} // namespace star_tracker
