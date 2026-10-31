/**
 * @file star.hpp
 * @brief Data structures for astronomical catalog stars and focal-plane image detections.
 *
 * @details
 * This file defines two fundamental data structures in star tracker processing:
 * 1. `Star`: An astronomical star from an inertial reference catalog (such as Hipparcos,
 *    Tycho-2, or Gaia), parameterized by its celestial coordinates:
 *    - Right Ascension (alpha): Longitude angle in equatorial plane [0, 2*pi).
 *    - Declination (delta): Latitude angle above/below celestial equator [-pi/2, +pi/2].
 *    - Visual Magnitude: Apparent stellar brightness scale (Pogson scale, where lower/negative
 *      numbers are brighter).
 *    Includes spherical-to-Cartesian unit vector transformations:
 *      v = [ cos(delta)*cos(alpha), cos(delta)*sin(alpha), sin(delta) ]^T
 *
 * 2. `ImageStar`: A 2D focal-plane star detection or projection, characterized by:
 *    - Subpixel centroid coordinates (x, y) on the CMOS/CCD image sensor.
 *    - Integrated pixel flux (total brightness) and estimated apparent magnitude.
 *    - Catalog ID (populated if identified).
 *
 * In Delabie et al. (2013, "SDT"), star magnitude plays a critical role in candidate generation:
 * bright stars are sorted to select the top combinations for triplet centroid feature matching.
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." JGCD, 36(2), pp. 476–488. Section II.B
 *   "Database Images" and Section II.C "Matching the Camera Image with the Database Images".
 *   DOI: 10.2514/1.56860.
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." JGCD, 36(6), pp. 1672–1680.
 *   DOI: 10.2514/1.61082.
 */

#pragma once

#include "math/vector.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace star_tracker {

/**
 * @struct Star
 * @brief Celestial catalog star in Earth-Centered Inertial (ECI J2000) coordinates.
 */
struct Star {
    uint64_t id{0};              ///< Unique catalog identifier (e.g. Hipparcos HIP number)

    double right_ascension{0.0}; ///< Right ascension alpha in radians [0, 2*pi)
    double declination{0.0};     ///< Declination delta in radians [-pi/2, +pi/2]
    double magnitude{0.0};       ///< Apparent visual magnitude (standard astronomical scale)

    /// Default constructor.
    constexpr Star() = default;

    /**
     * @brief Parameterized constructor.
     * @param id_ Unique catalog identifier.
     * @param ra Right ascension in radians.
     * @param dec Declination in radians.
     * @param mag Visual magnitude.
     */
    constexpr Star(uint64_t id_, double ra, double dec, double mag)
        : id(id_), right_ascension(ra), declination(dec), magnitude(mag) {}

    /**
     * @brief Converts celestial spherical coordinates (RA, Dec) to an ECI J2000 3D unit vector.
     * @details
     * v_x = cos(delta) * cos(alpha)
     * v_y = cos(delta) * sin(alpha)
     * v_z = sin(delta)
     * @return 3D Cartesian unit vector on celestial sphere.
     */
    Vector3 to_unit_vector() const {
        double cd = std::cos(declination);
        return Vector3(
            cd * std::cos(right_ascension),
            cd * std::sin(right_ascension),
            std::sin(declination)
        );
    }

    /**
     * @brief Constructs a Star from a 3D Cartesian unit vector.
     * @details
     * delta = asin(clamp(v_z, -1.0, 1.0))
     * alpha = atan2(v_y, v_x), normalized to [0, 2*pi)
     * @param id Unique catalog ID.
     * @param v 3D direction vector.
     * @param mag Apparent visual magnitude.
     */
    static Star from_unit_vector(uint64_t id, const Vector3& v, double mag = 0.0) {
        Vector3 u = v.normalized();
        double dec = std::asin(std::clamp(u.z, -1.0, 1.0));
        double ra = std::atan2(u.y, u.x);
        if (ra < 0.0) {
            ra += 2.0 * M_PI;
        }
        return Star(id, ra, dec, mag);
    }

    /**
     * @brief Computes relative stellar photometric flux (radiant intensity relative to magnitude 0).
     * @details
     * Pogson's relation: Flux = 10^(-0.4 * magnitude).
     */
    double brightness() const {
        return std::pow(10.0, -0.4 * magnitude);
    }
};

/**
 * @struct ImageStar
 * @brief Detected star spot or projected catalog star in 2D sensor focal-plane coordinates.
 */
struct ImageStar {
    uint64_t catalog_id{0}; ///< Associated catalog identifier (0 if unassociated)

    double x{0.0};          ///< Focal-plane horizontal pixel coordinate (subpixel centroid)
    double y{0.0};          ///< Focal-plane vertical pixel coordinate (subpixel centroid)

    double brightness{0.0}; ///< Integrated pixel intensity (flux) above background
    double magnitude{0.0};  ///< Estimated apparent visual magnitude

    /// Default constructor.
    constexpr ImageStar() = default;

    /**
     * @brief Parameterized constructor.
     * @param id Catalog star ID.
     * @param x_ Subpixel X coordinate.
     * @param y_ Subpixel Y coordinate.
     * @param b Measured integrated brightness/flux.
     * @param m Estimated or catalog visual magnitude.
     */
    constexpr ImageStar(uint64_t id, double x_, double y_, double b = 0.0, double m = 0.0)
        : catalog_id(id), x(x_), y(y_), brightness(b), magnitude(m) {}
};

} // namespace star_tracker
