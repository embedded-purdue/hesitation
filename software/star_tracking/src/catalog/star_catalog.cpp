/**
 * @file star_catalog.cpp
 * @brief Implementation of StarCatalog parsing, filtering, and perspective FOV projection.
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." Journal of Guidance, Control, and Dynamics, 36(2),
 *   pp. 476–488. DOI: 10.2514/1.56860.
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." Journal of Guidance, Control,
 *   and Dynamics, 36(6), pp. 1672–1680. DOI: 10.2514/1.61082.
 */

#include "catalog/star_catalog.hpp"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

namespace star_tracker {

bool StarCatalog::load_from_csv(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        // Fallback for execution from the build/ directory
        std::string fallback = "../" + filepath;
        file.open(fallback);
        if (!file.is_open()) {
            std::cerr << "Failed to open star catalog CSV: " << filepath << " (also tried " << fallback << ")\n";
            return false;
        }
    }

    stars_.clear();
    std::string line;

    // Check and consume header line if present
    if (std::getline(file, line)) {
        // Expected header format: id,right_ascension,declination,magnitude
    }

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> row;

        while (std::getline(ss, token, ',')) {
            row.push_back(token);
        }

        if (row.size() >= 4) {
            try {
                uint64_t id = std::stoull(row[0]);
                double ra = std::stod(row[1]);
                double dec = std::stod(row[2]);
                double mag = std::stod(row[3]);
                stars_.emplace_back(id, ra, dec, mag);
            } catch (const std::exception& e) {
                // Ignore parse errors on malformed lines
                continue;
            }
        }
    }

    std::cout << "Loaded " << stars_.size() << " stars from " << filepath << "\n";
    return !stars_.empty();
}

StarCatalog StarCatalog::filter_by_magnitude(double max_mag) const {
    StarCatalog filtered;
    for (const auto& star : stars_) {
        if (star.magnitude <= max_mag) {
            filtered.add_star(star);
        }
    }
    return filtered;
}

std::vector<ImageStar> StarCatalog::project_visible_stars(
    const Quaternion& attitude,
    const CameraModel& camera,
    double max_mag,
    double margin_pixels
) const {
    std::vector<ImageStar> visible;

    for (const auto& star : stars_) {
        if (star.magnitude > max_mag) continue;

        // Convert celestial coordinates to inertial unit direction vector
        Vector3 v_inertial = star.to_unit_vector();

        // Rotate into camera coordinate frame (+Z forward along optical axis)
        // v_camera = R(attitude) * v_inertial
        // (Delabie et al., 2013, AIM Eq. 5)
        Vector3 v_camera = attitude.rotate(v_inertial);

        // Discard stars that are behind the camera focal plane or perpendicular to it
        if (v_camera.z <= 1e-4) continue;

        double px = 0.0, py = 0.0;
        // Project through pinhole and lens distortion models
        if (camera.vector_to_pixel(v_camera, px, py)) {
            if (camera.is_in_bounds(px, py, margin_pixels)) {
                visible.emplace_back(star.id, px, py, star.brightness(), star.magnitude);
            }
        }
    }

    // Sort visible stars by brightness descending (astronomical magnitude ascending)
    // As noted in Delabie et al. (2013), matching operates primarily on the brightest stars
    std::sort(visible.begin(), visible.end(), [](const ImageStar& a, const ImageStar& b) {
        return a.magnitude < b.magnitude;
    });

    return visible;
}

const Star* StarCatalog::find_star(uint64_t id) const {
    for (const auto& star : stars_) {
        if (star.id == id) {
            return &star;
        }
    }
    return nullptr;
}

} // namespace star_tracker
