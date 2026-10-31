/**
 * @file star_detector.cpp
 * @brief Implementation of autonomous star detection via BFS connected components and adaptive thresholding.
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." Journal of Guidance, Control, and Dynamics, 36(2),
 *   pp. 476–488. DOI: 10.2514/1.56860.
 */

#include "detection/star_detector.hpp"
#include <algorithm>
#include <cmath>
#include <queue>

namespace star_tracker {

std::vector<ImageStar> StarDetector::detect_stars(
    const Image& image,
    const CameraModel& camera
) const {
    const int width = image.width();
    const int height = image.height();
    if (width <= 0 || height <= 0) return {};

    // Stage 1: Determine detection threshold
    double threshold = config_.threshold;
    if (config_.adaptive_threshold) {
        // Calculate image mean and standard deviation for robust constant-false-alarm-rate (CFAR)
        double sum = 0.0;
        double sum_sq = 0.0;
        const int total_pixels = width * height;
        const uint8_t* data = image.data();

        for (int i = 0; i < total_pixels; ++i) {
            double v = data[i];
            sum += v;
            sum_sq += v * v;
        }

        double mean = sum / total_pixels;
        double variance = std::max(0.0, (sum_sq / total_pixels) - (mean * mean));
        double std_dev = std::sqrt(variance);

        // Threshold set at mu + k * sigma above background floor
        threshold = mean + config_.noise_sigma_mult * std_dev;
    }

    // Stage 2: Connected Component Labeling via 8-connectivity Breadth-First Search (BFS)
    std::vector<bool> visited(width * height, false);
    std::vector<ImageStar> detected;

    // 8-neighborhood offsets (dx, dy)
    static const int dx8[8] = {-1,  0,  1, -1, 1, -1, 0, 1};
    static const int dy8[8] = {-1, -1, -1,  0, 0,  1, 1, 1};

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = y * width + x;
            if (visited[idx] || image.at(x, y) < threshold) {
                continue;
            }

            // Initiate BFS search for current connected component
            std::vector<PixelCoord> region;
            std::queue<std::pair<int, int>> q;

            visited[idx] = true;
            q.push({x, y});

            while (!q.empty()) {
                auto [cx, cy] = q.front();
                q.pop();

                double intensity = image.at(cx, cy);
                region.push_back({cx, cy, intensity});

                // Explore 8 neighboring pixels
                for (int d = 0; d < 8; ++d) {
                    int nx = cx + dx8[d];
                    int ny = cy + dy8[d];

                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        int nidx = ny * width + nx;
                        if (!visited[nidx] && image.at(nx, ny) >= threshold) {
                            visited[nidx] = true;
                            q.push({nx, ny});
                        }
                    }
                }
            }

            // Stage 3: Filter components by physical pixel footprint size
            if (static_cast<int>(region.size()) < config_.min_region_size ||
                static_cast<int>(region.size()) > config_.max_region_size) {
                continue;
            }

            // Stage 4: Extract subpixel centroid coordinates and integrated flux
            ImageStar star = CentroidExtractor::extract_centroid(region, threshold);
            if (star.brightness >= config_.min_total_flux) {
                // Ensure detected centroid is strictly within valid camera FOV
                if (camera.is_in_bounds(star.x, star.y)) {
                    detected.push_back(star);
                }
            }
        }
    }

    // Stage 5: Sort detected stars by brightness descending (brightest first)
    // In Delabie et al. (2013), matching evaluates combinations among the top brightest stars
    std::sort(detected.begin(), detected.end(), [](const ImageStar& a, const ImageStar& b) {
        return a.brightness > b.brightness;
    });

    return detected;
}

} // namespace star_tracker
