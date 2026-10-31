/**
 * @file reference_database.cpp
 * @brief Implementation of binary reference database serialization and metadata checks.
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." Journal of Guidance, Control, and Dynamics, 36(2),
 *   pp. 476–488. DOI: 10.2514/1.56860.
 */

#include "database/reference_database.hpp"
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>

namespace star_tracker {

// Magic header bytes identifying the binary database file format: "STRKDB01"
static const char MAGIC[8] = {'S', 'T', 'R', 'K', 'D', 'B', '0', '1'};

bool ReferenceDatabase::save_binary(const std::string& filepath, const DatabaseMetadata& meta) const {
    std::ofstream out(filepath, std::ios::binary);
    if (!out.is_open()) return false;

    // Write magic signature
    out.write(MAGIC, 8);

    // Write camera intrinsic and catalog metadata
    uint32_t ver = meta.format_version;
    uint32_t num_imgs = static_cast<uint32_t>(images_.size());
    int32_t w = meta.camera_width;
    int32_t h = meta.camera_height;
    double fl = meta.focal_length;
    double hfov = meta.horizontal_fov;
    double vfov = meta.vertical_fov;
    double k1 = meta.distortion_k1;
    double k2 = meta.distortion_k2;
    double mag_th = meta.magnitude_threshold;

    out.write(reinterpret_cast<const char*>(&ver), sizeof(ver));
    out.write(reinterpret_cast<const char*>(&num_imgs), sizeof(num_imgs));
    out.write(reinterpret_cast<const char*>(&w), sizeof(w));
    out.write(reinterpret_cast<const char*>(&h), sizeof(h));
    out.write(reinterpret_cast<const char*>(&fl), sizeof(fl));
    out.write(reinterpret_cast<const char*>(&hfov), sizeof(hfov));
    out.write(reinterpret_cast<const char*>(&vfov), sizeof(vfov));
    out.write(reinterpret_cast<const char*>(&k1), sizeof(k1));
    out.write(reinterpret_cast<const char*>(&k2), sizeof(k2));
    out.write(reinterpret_cast<const char*>(&mag_th), sizeof(mag_th));

    // Write each reference image
    for (const auto& img : images_) {
        uint64_t id = img.id;
        out.write(reinterpret_cast<const char*>(&id), sizeof(id));

        // Attitude quaternion components [w, x, y, z]
        double att[4] = {img.attitude.w, img.attitude.x, img.attitude.y, img.attitude.z};
        out.write(reinterpret_cast<const char*>(att), sizeof(att));

        // Precomputed Delabie centroid feature combinations
        uint32_t n_feats = static_cast<uint32_t>(img.feature_combinations.size());
        out.write(reinterpret_cast<const char*>(&n_feats), sizeof(n_feats));

        for (const auto& feat : img.feature_combinations) {
            uint8_t f_valid = feat.valid ? 1 : 0;
            out.write(reinterpret_cast<const char*>(&f_valid), sizeof(f_valid));
            out.write(reinterpret_cast<const char*>(&feat.centroid_x), sizeof(double));
            out.write(reinterpret_cast<const char*>(&feat.centroid_y), sizeof(double));
            out.write(reinterpret_cast<const char*>(feat.distances.data()), sizeof(double) * 3);
            out.write(reinterpret_cast<const char*>(feat.angles.data()), sizeof(double) * 3);
            out.write(reinterpret_cast<const char*>(&feat.orientation_angle), sizeof(double));
            out.write(reinterpret_cast<const char*>(feat.star_ids.data()), sizeof(uint64_t) * 3);
        }

        // Projected visible catalog stars
        uint32_t n_stars = static_cast<uint32_t>(img.stars.size());
        out.write(reinterpret_cast<const char*>(&n_stars), sizeof(n_stars));

        for (const auto& s : img.stars) {
            out.write(reinterpret_cast<const char*>(&s.catalog_id), sizeof(s.catalog_id));
            out.write(reinterpret_cast<const char*>(&s.x), sizeof(s.x));
            out.write(reinterpret_cast<const char*>(&s.y), sizeof(s.y));
            out.write(reinterpret_cast<const char*>(&s.brightness), sizeof(s.brightness));
            out.write(reinterpret_cast<const char*>(&s.magnitude), sizeof(s.magnitude));
        }
    }

    return out.good();
}

bool ReferenceDatabase::load_binary(const std::string& filepath, DatabaseMetadata* meta) {
    std::ifstream in(filepath, std::ios::binary);
    if (!in.is_open()) {
        std::string fallback = "../" + filepath;
        in.open(fallback, std::ios::binary);
        if (!in.is_open()) return false;
    }

    char magic[8];
    in.read(magic, 8);
    if (std::memcmp(magic, MAGIC, 8) != 0) {
        std::cerr << "Invalid database binary magic: file is not a valid STRKDB01 database\n";
        return false;
    }

    uint32_t ver, num_imgs;
    int32_t w, h;
    double fl, hfov, vfov, k1, k2, mag_th;

    in.read(reinterpret_cast<char*>(&ver), sizeof(ver));
    in.read(reinterpret_cast<char*>(&num_imgs), sizeof(num_imgs));
    in.read(reinterpret_cast<char*>(&w), sizeof(w));
    in.read(reinterpret_cast<char*>(&h), sizeof(h));
    in.read(reinterpret_cast<char*>(&fl), sizeof(fl));
    in.read(reinterpret_cast<char*>(&hfov), sizeof(hfov));
    in.read(reinterpret_cast<char*>(&vfov), sizeof(vfov));
    in.read(reinterpret_cast<char*>(&k1), sizeof(k1));
    in.read(reinterpret_cast<char*>(&k2), sizeof(k2));
    in.read(reinterpret_cast<char*>(&mag_th), sizeof(mag_th));

    metadata_.format_version = ver;
    metadata_.num_images = num_imgs;
    metadata_.camera_width = w;
    metadata_.camera_height = h;
    metadata_.focal_length = fl;
    metadata_.horizontal_fov = hfov;
    metadata_.vertical_fov = vfov;
    metadata_.distortion_k1 = k1;
    metadata_.distortion_k2 = k2;
    metadata_.magnitude_threshold = mag_th;

    if (meta) {
        *meta = metadata_;
    }

    images_.clear();
    images_.reserve(num_imgs);

    for (uint32_t i = 0; i < num_imgs; ++i) {
        ReferenceImage img;
        in.read(reinterpret_cast<char*>(&img.id), sizeof(img.id));

        double att[4];
        in.read(reinterpret_cast<char*>(att), sizeof(att));
        img.attitude = Quaternion(att[0], att[1], att[2], att[3]);

        uint32_t n_feats;
        in.read(reinterpret_cast<char*>(&n_feats), sizeof(n_feats));
        img.feature_combinations.resize(n_feats);

        for (uint32_t j = 0; j < n_feats; ++j) {
            auto& feat = img.feature_combinations[j];
            uint8_t f_valid;
            in.read(reinterpret_cast<char*>(&f_valid), sizeof(f_valid));
            feat.valid = (f_valid != 0);
            in.read(reinterpret_cast<char*>(&feat.centroid_x), sizeof(double));
            in.read(reinterpret_cast<char*>(&feat.centroid_y), sizeof(double));
            in.read(reinterpret_cast<char*>(feat.distances.data()), sizeof(double) * 3);
            in.read(reinterpret_cast<char*>(feat.angles.data()), sizeof(double) * 3);
            in.read(reinterpret_cast<char*>(&feat.orientation_angle), sizeof(double));
            in.read(reinterpret_cast<char*>(feat.star_ids.data()), sizeof(uint64_t) * 3);
        }

        if (!img.feature_combinations.empty()) {
            img.centroid_features = img.feature_combinations[0];
        } else {
            img.centroid_features.valid = false;
        }

        uint32_t n_stars;
        in.read(reinterpret_cast<char*>(&n_stars), sizeof(n_stars));
        img.stars.resize(n_stars);

        for (uint32_t s = 0; s < n_stars; ++s) {
            auto& st = img.stars[s];
            in.read(reinterpret_cast<char*>(&st.catalog_id), sizeof(st.catalog_id));
            in.read(reinterpret_cast<char*>(&st.x), sizeof(st.x));
            in.read(reinterpret_cast<char*>(&st.y), sizeof(st.y));
            in.read(reinterpret_cast<char*>(&st.brightness), sizeof(st.brightness));
            in.read(reinterpret_cast<char*>(&st.magnitude), sizeof(st.magnitude));
        }

        images_.push_back(std::move(img));
    }

    return in.good() || in.eof();
}

bool ReferenceDatabase::save_metadata_json(const std::string& filepath, const DatabaseMetadata& meta) const {
    std::ofstream out(filepath);
    if (!out.is_open()) return false;

    out << "{\n"
        << "  \"format_version\": " << meta.format_version << ",\n"
        << "  \"catalog_source\": \"" << meta.catalog_source << "\",\n"
        << "  \"camera_width\": " << meta.camera_width << ",\n"
        << "  \"camera_height\": " << meta.camera_height << ",\n"
        << "  \"horizontal_fov_rad\": " << meta.horizontal_fov << ",\n"
        << "  \"vertical_fov_rad\": " << meta.vertical_fov << ",\n"
        << "  \"focal_length_pixels\": " << meta.focal_length << ",\n"
        << "  \"distortion_k1\": " << meta.distortion_k1 << ",\n"
        << "  \"distortion_k2\": " << meta.distortion_k2 << ",\n"
        << "  \"magnitude_threshold\": " << meta.magnitude_threshold << ",\n"
        << "  \"num_images\": " << meta.num_images << "\n"
        << "}\n";

    return out.good();
}

bool ReferenceDatabase::load_metadata_json(const std::string& filepath, DatabaseMetadata& meta) const {
    std::ifstream in(filepath);
    if (!in.is_open()) return false;

    std::string line;
    while (std::getline(in, line)) {
        auto parse_val = [&line](const std::string& key) -> std::string {
            size_t pos = line.find("\"" + key + "\"");
            if (pos == std::string::npos) return "";
            size_t colon = line.find(':', pos);
            if (colon == std::string::npos) return "";
            size_t val_start = line.find_first_not_of(" \t\"", colon + 1);
            size_t val_end = line.find_last_not_of(" \t\",\r\n");
            if (val_start != std::string::npos && val_end != std::string::npos && val_end >= val_start) {
                return line.substr(val_start, val_end - val_start + 1);
            }
            return "";
        };

        std::string v;
        if (!(v = parse_val("camera_width")).empty()) meta.camera_width = std::stoi(v);
        else if (!(v = parse_val("camera_height")).empty()) meta.camera_height = std::stoi(v);
        else if (!(v = parse_val("focal_length_pixels")).empty()) meta.focal_length = std::stod(v);
        else if (!(v = parse_val("horizontal_fov_rad")).empty()) meta.horizontal_fov = std::stod(v);
        else if (!(v = parse_val("vertical_fov_rad")).empty()) meta.vertical_fov = std::stod(v);
        else if (!(v = parse_val("magnitude_threshold")).empty()) meta.magnitude_threshold = std::stod(v);
        else if (!(v = parse_val("num_images")).empty()) meta.num_images = std::stoul(v);
    }
    return true;
}

bool ReferenceDatabase::is_compatible(
    const CameraModel& camera,
    const DatabaseMetadata& meta,
    double tolerance
) const {
    if (camera.width != meta.camera_width || camera.height != meta.camera_height) {
        return false;
    }
    double fl_err = std::abs(camera.focal_length - meta.focal_length) / meta.focal_length;
    if (fl_err > tolerance) return false;

    double hfov_err = std::abs(camera.horizontal_fov - meta.horizontal_fov);
    if (hfov_err > tolerance) return false;

    return true;
}

const ReferenceImage* ReferenceDatabase::find_image(uint64_t id) const {
    for (const auto& img : images_) {
        if (img.id == id) return &img;
    }
    return nullptr;
}

} // namespace star_tracker
