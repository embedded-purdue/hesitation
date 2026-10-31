/**
 * @file image.cpp
 * @brief Implementation of Image PGM I/O and Gaussian PSF synthetic star field rendering.
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." Journal of Guidance, Control, and Dynamics, 36(2),
 *   pp. 476–488. DOI: 10.2514/1.56860.
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." Journal of Guidance, Control,
 *   and Dynamics, 36(6), pp. 1672–1680. DOI: 10.2514/1.61082.
 */

#include "detection/image.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>

namespace star_tracker {

bool Image::save_pgm(const std::string& filepath) const {
    std::ofstream out(filepath, std::ios::binary);
    if (!out.is_open()) return false;

    // Standard P5 binary header: magic, width, height, max_val
    out << "P5\n" << width_ << " " << height_ << "\n255\n";
    out.write(reinterpret_cast<const char*>(data_.data()), data_.size());
    return out.good();
}

bool Image::load_pgm(const std::string& filepath) {
    std::ifstream in(filepath, std::ios::binary);
    if (!in.is_open()) return false;

    std::string magic;
    in >> magic;
    if (magic != "P5") return false;

    // Skip comment lines starting with '#'
    auto skip_comments = [&in]() {
        while (in >> std::ws && in.peek() == '#') {
            std::string line;
            std::getline(in, line);
        }
    };

    skip_comments();
    int w, h, max_val;
    in >> w >> h;
    skip_comments();
    in >> max_val;
    in.get(); // Consume single trailing whitespace / newline

    if (w <= 0 || h <= 0 || max_val <= 0 || max_val > 255) return false;

    width_ = w;
    height_ = h;
    data_.resize(w * h);
    in.read(reinterpret_cast<char*>(data_.data()), data_.size());
    return in.good();
}

Image Image::render_star_field(
    int width,
    int height,
    const std::vector<ImageStar>& stars,
    double psf_sigma,
    double background_noise_sigma,
    uint8_t background_bias
) {
    std::vector<float> buffer(width * height, static_cast<float>(background_bias));

    // Seeded Mersenne Twister for deterministic simulation testing
    std::mt19937 gen(42);
    std::normal_distribution<float> noise(0.0f, static_cast<float>(background_noise_sigma));

    // Add sensor noise (read noise + thermal dark current fluctuation)
    if (background_noise_sigma > 0.0) {
        for (auto& val : buffer) {
            val += noise(gen);
        }
    }

    // Gaussian point spread function (PSF) kernel calculation
    // Truncate kernel at 3 * sigma where intensity drops to exp(-4.5) ≈ 0.011 of peak
    int radius = std::max(2, static_cast<int>(std::ceil(3.0 * psf_sigma)));
    double two_sigma_sq = 2.0 * psf_sigma * psf_sigma;
    double norm_factor = 1.0 / (2.0 * M_PI * psf_sigma * psf_sigma);

    for (const auto& star : stars) {
        // Convert visual magnitude to sensor counts:
        // Pogson's law: Flux = 10^(-0.4 * (mag - mag_0)) * scale
        double total_flux = 10000.0 * std::pow(10.0, -0.4 * (star.magnitude - 2.0));
        if (star.brightness > 0.0 && std::abs(star.magnitude) < 1e-6) {
            total_flux = star.brightness * 63095.7;
        }
        total_flux = std::clamp(total_flux, 50.0, 50000.0);

        // Compute integer bounding box for the star spot
        int min_x = std::max(0, static_cast<int>(std::floor(star.x - radius)));
        int max_x = std::min(width - 1, static_cast<int>(std::ceil(star.x + radius)));
        int min_y = std::max(0, static_cast<int>(std::floor(star.y - radius)));
        int max_y = std::min(height - 1, static_cast<int>(std::ceil(star.y + radius)));

        for (int y = min_y; y <= max_y; ++y) {
            double dy = y - star.y;
            for (int x = min_x; x <= max_x; ++x) {
                double dx = x - star.x;
                double dist_sq = dx * dx + dy * dy;
                double val = total_flux * norm_factor * std::exp(-dist_sq / two_sigma_sq);
                buffer[y * width + x] += static_cast<float>(val);
            }
        }
    }

    // Convert accumulated floating-point intensities into 8-bit sensor output with clamping [0, 255]
    Image result(width, height);
    for (int i = 0; i < width * height; ++i) {
        float v = std::clamp(buffer[i], 0.0f, 255.0f);
        result.data()[i] = static_cast<uint8_t>(std::round(v));
    }

    return result;
}

} // namespace star_tracker
