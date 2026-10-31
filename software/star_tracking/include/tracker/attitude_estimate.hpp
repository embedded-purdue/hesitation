/**
 * @file attitude_estimate.hpp
 * @brief Spacecraft attitude estimation output, confidence, and telemetry representation.
 *
 * @details
 * This structure encapsulates the 3-axis attitude determination solution output by the
 * star tracker pipeline (from either ACQUISITION, TRACKING, or RECOVERY mode).
 *
 * It contains:
 * - Quaternion attitude: Unit quaternion representing the rotation from Earth-Centered
 *   Inertial (ECI J2000) coordinates to the camera sensor frame (or spacecraft body frame).
 * - Confidence: Normalized quality score [0.0, 1.0] derived from matching residuals and
 *   close star ratios (Delabie et al., 2013).
 * - Valid flag: Indicates whether all geometric, photometric, and residual error checks passed.
 * - Diagnostic metrics: Matched star count, RMS residual pixel error, and timestamp.
 *
 * @reference
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." JGCD, 36(6), pp. 1672–1680.
 *   DOI: 10.2514/1.61082.
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." JGCD, 36(2), pp. 476–488. DOI: 10.2514/1.56860.
 */

#pragma once

#include "math/quaternion.hpp"
#include <cstdint>

namespace star_tracker {

/**
 * @struct AttitudeEstimate
 * @brief Standardized attitude solution and quality metrics produced by the star tracker.
 */
struct AttitudeEstimate {
    /// Estimated unit quaternion mapping celestial inertial frame (J2000) to camera sensor frame
    Quaternion attitude{Quaternion::identity()};

    /// Normalized estimation confidence metric [0.0 = untrusted / lost, 1.0 = optimal lock]
    double confidence{0.0};

    /// Status flag: true if attitude solution passed all gating and residual checks
    bool valid{false};

    /// Number of stars successfully matched and used in the attitude calculation
    int matched_stars{0};

    /// Root-Mean-Square (RMS) focal plane residual error in pixels
    double rms_error_pixels{0.0};

    /// Frame acquisition timestamp in nanoseconds (for time synchronization in HITL)
    uint64_t timestamp_ns{0};

    /// Default constructor.
    constexpr AttitudeEstimate() = default;

    /**
     * @brief Baseline constructor for attitude, confidence, and validity.
     * @param att Estimated quaternion.
     * @param conf Confidence score [0, 1].
     * @param val Validity flag.
     */
    constexpr AttitudeEstimate(const Quaternion& att, double conf, bool val)
        : attitude(att), confidence(conf), valid(val), matched_stars(0), rms_error_pixels(0.0), timestamp_ns(0) {}

    /**
     * @brief Full parameter constructor.
     * @param att Estimated quaternion.
     * @param conf Confidence score.
     * @param val Validity flag.
     * @param n_matched Number of matched stars.
     * @param rms RMS residual error in pixels.
     * @param ts Timestamp in nanoseconds.
     */
    constexpr AttitudeEstimate(const Quaternion& att, double conf, bool val,
                               int n_matched, double rms, uint64_t ts = 0)
        : attitude(att), confidence(conf), valid(val),
          matched_stars(n_matched), rms_error_pixels(rms), timestamp_ns(ts) {}
};

} // namespace star_tracker
