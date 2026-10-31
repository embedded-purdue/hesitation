/**
 * @file distance_transform.cpp
 * @brief Implementation of Maurer's linear-time exact Euclidean distance transform.
 *
 * @reference
 * - Maurer, C. R., Qi, R., and Raghavan, V. (2003). "A Linear Time Algorithm for Computing
 *   Exact Euclidean Distance Transforms of Binary Images in Arbitrary Dimensions." IEEE TPAMI,
 *   25(2), pp. 265–270. DOI: 10.1109/TPAMI.2003.1177156.
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." JGCD, 36(2), pp. 476–488. DOI: 10.2514/1.56860.
 */

#include "lost_in_space/distance_transform.hpp"
#include <algorithm>
#include <cmath>

namespace star_tracker {

DistanceTransform::DistanceTransform(int width, int height)
    : width_(width), height_(height), distances_(width * height, INF_DIST) {}

void DistanceTransform::compute(
    const std::vector<ImageStar>& stars,
    int width,
    int height
) {
    width_ = width;
    height_ = height;
    distances_.assign(width * height, INF_DIST);

    if (stars.empty() || width <= 0 || height <= 0) {
        return;
    }

    // Pass 1: Bucket star centroids by column x
    std::vector<uint32_t> col_d1(width * height, INF_DIST);
    std::vector<std::vector<int>> stars_in_col(width);

    for (const auto& star : stars) {
        int sx = static_cast<int>(std::round(star.x));
        int sy = static_cast<int>(std::round(star.y));
        if (sx >= 0 && sx < width && sy >= 0 && sy < height) {
            stars_in_col[sx].push_back(sy);
        }
    }

    // Step 1: 1D distance transform along each column (dimension y)
    for (int x = 0; x < width; ++x) {
        if (stars_in_col[x].empty()) {
            continue;
        }
        std::sort(stars_in_col[x].begin(), stars_in_col[x].end());

        // Linear scan across rows to find closest vertical star in column x
        int star_idx = 0;
        int num_col_stars = static_cast<int>(stars_in_col[x].size());

        for (int y = 0; y < height; ++y) {
            while (star_idx + 1 < num_col_stars &&
                   std::abs(stars_in_col[x][star_idx + 1] - y) <= std::abs(stars_in_col[x][star_idx] - y)) {
                star_idx++;
            }
            int dy = y - stars_in_col[x][star_idx];
            col_d1[y * width + x] = static_cast<uint32_t>(dy * dy);
        }
    }

    // Step 2: 1D distance transform along each row (dimension x) using Maurer's parabolic envelope
    // Each star column i acts as a 1D parabola: f_i(x) = (x - i)^2 + col_d1[i]
    // The lower envelope of these parabolas defines the Voronoi partitioning along the row
    std::vector<int> s(width);    // Stack of active parabola column indices
    std::vector<double> t(width); // Boundary intersection points between adjacent parabolas

    for (int y = 0; y < height; ++y) {
        int row_offset = y * width;

        int k = -1; // Stack pointer
        for (int i = 0; i < width; ++i) {
            uint32_t fi = col_d1[row_offset + i];
            if (fi >= INF_DIST) continue;

            if (k < 0) {
                // Initialize stack with first valid parabola
                k = 0;
                s[0] = i;
                t[0] = -1e9;
                continue;
            }

            // Find horizontal intersection between parabola sk and new parabola i:
            // (t - sk)^2 + f_sk = (t - i)^2 + fi
            // t^2 - 2*t*sk + sk^2 + f_sk = t^2 - 2*t*i + i^2 + fi
            // 2*t*(i - sk) = (i^2 - sk^2 + fi - f_sk)
            // t = (i^2 - sk^2 + fi - f_sk) / (2 * (i - sk))
            while (k >= 0) {
                int sk = s[k];
                uint32_t f_sk = col_d1[row_offset + sk];

                double num = static_cast<double>(i * i - sk * sk) +
                             static_cast<double>(fi) - static_cast<double>(f_sk);
                double den = 2.0 * static_cast<double>(i - sk);
                double t_new = num / den;

                if (k == 0) {
                    k = 1;
                    s[1] = i;
                    t[1] = t_new;
                    break;
                } else if (t_new > t[k]) {
                    k++;
                    s[k] = i;
                    t[k] = t_new;
                    break;
                } else {
                    // Parabola s[k] is completely obscured by parabola i, pop it
                    k--;
                }
            }
        }

        if (k < 0) {
            // Row has no star influence
            continue;
        }

        // Query lower envelope for all horizontal pixel locations x in [0, width - 1]
        int ptr = 0;
        for (int x = 0; x < width; ++x) {
            while (ptr < k && t[ptr + 1] <= static_cast<double>(x)) {
                ptr++;
            }
            int best_col = s[ptr];
            int dx = x - best_col;
            uint32_t dist_sq = static_cast<uint32_t>(dx * dx) + col_d1[row_offset + best_col];
            distances_[row_offset + x] = dist_sq;
        }
    }
}

void DistanceTransform::compute_brute_force(
    const std::vector<ImageStar>& stars,
    int width,
    int height
) {
    width_ = width;
    height_ = height;
    distances_.assign(width * height, INF_DIST);

    if (stars.empty()) return;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            uint32_t min_d2 = INF_DIST;
            for (const auto& s : stars) {
                int sx = static_cast<int>(std::round(s.x));
                int sy = static_cast<int>(std::round(s.y));
                int dx = x - sx;
                int dy = y - sy;
                uint32_t d2 = static_cast<uint32_t>(dx * dx + dy * dy);
                if (d2 < min_d2) min_d2 = d2;
            }
            distances_[y * width + x] = min_d2;
        }
    }
}

} // namespace star_tracker
