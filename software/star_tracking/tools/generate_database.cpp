/**
 * @file generate_database.cpp
 * @brief Standalone offline CLI tool for generating the star tracker reference database.
 *
 * @details
 * This utility generates the precomputed binary reference database (`reference_database.bin`)
 * and metadata JSON (`database_metadata.json`) used by the star tracker during flight.
 *
 * It takes an astronomical catalog (such as Hipparcos CSV), generates N pointing directions
 * covering the celestial sphere evenly via the Fibonacci spiral method (Delabie et al., 2013,
 * Section II.B.1), projects visible stars brighter than the magnitude cutoff (default: 5.5),
 * and precomputes all 6-feature centroid combinations for fast runtime filtering (Section II.E.2).
 *
 * Usage:
 * @code
 *   ./generate_database [catalog.csv] [output_database.bin] [output_metadata.json]
 * @endcode
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." Journal of Guidance, Control, and Dynamics, 36(2),
 *   pp. 476–488. DOI: 10.2514/1.56860.
 */

#include "camera/camera_model.hpp"
#include "catalog/star_catalog.hpp"
#include "database/database_generator.hpp"
#include "database/reference_database.hpp"
#include <iostream>

using namespace star_tracker;

int main(int argc, char** argv) {
    std::string catalog_path = "data/catalog/hipparcos.csv";
    std::string db_bin_path = "data/generated/reference_database.bin";
    std::string db_json_path = "data/generated/database_metadata.json";

    if (argc > 1) catalog_path = argv[1];
    if (argc > 2) db_bin_path = argv[2];
    if (argc > 3) db_json_path = argv[3];

    std::cout << "===========================================\n";
    std::cout << "Star Tracker Reference Database Generator\n";
    std::cout << "===========================================\n";
    std::cout << "Catalog:         " << catalog_path << "\n";
    std::cout << "Output Binary:   " << db_bin_path << "\n";
    std::cout << "Output JSON:     " << db_json_path << "\n";

    // 1. Ingest celestial star catalog
    StarCatalog catalog;
    if (!catalog.load_from_csv(catalog_path)) {
        std::cerr << "Error: Failed to load star catalog from " << catalog_path << "\n";
        return 1;
    }

    // 2. Camera configuration: 20x20 deg FOV, 1024x1024 pixels (Delabie et al. 2013)
    double fov_rad = 20.0 * M_PI / 180.0;
    CameraModel camera = CameraModel::from_fov(1024, 1024, fov_rad, fov_rad);

    // 3. Database generator configuration
    DatabaseGeneratorConfig config;
    config.num_images = 300;
    config.magnitude_threshold = 5.5; // Guarantees >= 3 stars visible across 100% of the sky
    config.catalog_source = "Hipparcos";

    DatabaseGenerator generator(config);
    ReferenceDatabase db = generator.generate(catalog, camera);

    // 4. Save metadata and binary file
    DatabaseMetadata meta;
    meta.format_version = 1;
    meta.catalog_source = config.catalog_source;
    meta.camera_width = camera.width;
    meta.camera_height = camera.height;
    meta.focal_length = camera.focal_length;
    meta.horizontal_fov = camera.horizontal_fov;
    meta.vertical_fov = camera.vertical_fov;
    meta.distortion_k1 = camera.distortion.k1;
    meta.distortion_k2 = camera.distortion.k2;
    meta.magnitude_threshold = config.magnitude_threshold;
    meta.num_images = static_cast<uint32_t>(db.size());

    if (!db.save_binary(db_bin_path, meta)) {
        std::cerr << "Error saving database binary to " << db_bin_path << "\n";
        return 1;
    }
    std::cout << "Successfully saved binary database to " << db_bin_path << "\n";

    if (!db.save_metadata_json(db_json_path, meta)) {
        std::cerr << "Error saving metadata JSON to " << db_json_path << "\n";
        return 1;
    }
    std::cout << "Successfully saved metadata JSON to " << db_json_path << "\n";

    // 5. Verification round-trip load
    ReferenceDatabase loaded;
    DatabaseMetadata loaded_meta;
    if (!loaded.load_binary(db_bin_path, &loaded_meta)) {
        std::cerr << "Error: Verification load failed!\n";
        return 1;
    }
    std::cout << "Verification successful: loaded " << loaded.size() << " reference images.\n";
    std::cout << "===========================================\n";

    return 0;
}
