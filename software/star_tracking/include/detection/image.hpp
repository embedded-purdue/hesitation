/**
 * @file image.hpp
 * @brief 8-bit grayscale image container, PGM file I/O, and synthetic star field rendering.
 *
 * @details
 * This class provides image buffer management for the star tracker optical pipeline.
 *
 * Capabilities:
 * 1. Pixel Storage: Contiguous 8-bit unsigned char buffer (row-major: y * width + x),
 *    compatible with direct DMA transfers from camera hardware (CameraLink, USB3, MIPI, GigE).
 * 2. PGM (Portable Graymap) I/O: Format P5 binary and ASCII file serialization for simulation,
 *    telemetry logging, and ground support equipment (GSE) verification.
 * 3. Synthetic Star Field Rendering: Generates high-fidelity synthetic camera frames modeling:
 *    - Diffraction-limited optical Point Spread Function (PSF) using a 2D Gaussian profile:
 *        I(x, y) = I_0 * exp( -((x - x_0)^2 + (y - y_0)^2) / (2 * sigma_psf^2) )
 *    - CMOS sensor readout noise and photon shot noise (Gaussian noise model).
 *    - Constant background bias (dark current / stray light offset).
 *
 * In HITL testing, `render_star_field` enables simulating realistic camera streams under
 * arbitrary spacecraft slew maneuvers without requiring physical optical collimators.
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." JGCD, 36(2), pp. 476–488. DOI: 10.2514/1.56860.
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." JGCD, 36(6), pp. 1672–1680.
 *   DOI: 10.2514/1.61082.
 */

#pragma once

#include "catalog/star.hpp"
#include <cstdint>
#include <string>
#include <vector>

namespace star_tracker {

/**
 * @class Image
 * @brief Monochromatic 8-bit image buffer for star tracker camera frames.
 */
class Image {
public:
    /// Default constructor.
    Image() = default;

    /**
     * @brief Constructs an image of given dimensions initialized to a uniform value.
     * @param width Image width in pixels.
     * @param height Image height in pixels.
     * @param initial_val Initial pixel intensity [0, 255] (default: 0).
     */
    Image(int width, int height, uint8_t initial_val = 0)
        : width_(width), height_(height), data_(width * height, initial_val) {}

    /// Returns horizontal resolution in pixels.
    int width() const { return width_; }

    /// Returns vertical resolution in pixels.
    int height() const { return height_; }

    /// Returns total number of pixels (width * height).
    size_t size() const { return data_.size(); }

    /// Read-only pointer to underlying contiguous byte buffer.
    const uint8_t* data() const { return data_.data(); }

    /// Mutable pointer to underlying contiguous byte buffer (useful for hardware DMA buffer copy).
    uint8_t* data() { return data_.data(); }

    /**
     * @brief Read-only pixel access with bounds checking.
     * Returns 0 if coordinates lie outside image boundaries.
     */
    uint8_t at(int x, int y) const {
        if (x < 0 || x >= width_ || y < 0 || y >= height_) return 0;
        return data_[y * width_ + x];
    }

    /**
     * @brief Mutable pixel reference without bounds check for high-performance inner loops.
     */
    uint8_t& at(int x, int y) {
        return data_[y * width_ + x];
    }

    /**
     * @brief Sets all pixels to a uniform scalar intensity.
     * @param val Pixel value [0, 255].
     */
    void fill(uint8_t val) {
        std::fill(data_.begin(), data_.end(), val);
    }

    /**
     * @brief Saves image to disk as a binary PGM (P5 format) file.
     * @param filepath Target file path.
     * @return true if successful, false on I/O error.
     */
    bool save_pgm(const std::string& filepath) const;

    /**
     * @brief Loads image from a binary PGM (P5 format) file.
     * @param filepath Source file path.
     * @return true if successful, false on I/O or parsing error.
     */
    bool load_pgm(const std::string& filepath);

    /**
     * @brief Renders a synthetic camera image containing simulated star spots with noise.
     * @details
     * Generates a 2D Gaussian point spread function (PSF) for each star:
     *   I(x, y) = I_0 * exp( -((x - x_0)^2 + (y - y_0)^2) / (2 * psf_sigma^2) )
     * where total flux I_0 is determined from visual magnitude. Adds zero-mean Gaussian
     * noise (standard deviation = background_noise_sigma) and a constant dark level bias.
     *
     * @param width Image width in pixels.
     * @param height Image height in pixels.
     * @param stars Vector of star coordinates and brightnesses.
     * @param psf_sigma Gaussian PSF radius standard deviation in pixels (default: 1.0).
     * @param background_noise_sigma Readout/shot noise standard deviation in counts (default: 2.0).
     * @param background_bias Constant dark level offset (default: 10 counts).
     * @return Synthesized Image frame.
     */
    static Image render_star_field(
        int width,
        int height,
        const std::vector<ImageStar>& stars,
        double psf_sigma = 1.0,
        double background_noise_sigma = 2.0,
        uint8_t background_bias = 10
    );

private:
    int width_{0};               ///< Image width (columns)
    int height_{0};              ///< Image height (rows)
    std::vector<uint8_t> data_;  ///< Contiguous 8-bit row-major pixel storage
};

} // namespace star_tracker
