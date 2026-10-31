/**
 * @file camera_model.hpp
 * @brief Pinhole camera model, projection geometry, and field-of-view testing for star trackers.
 *
 * @details
 * This file encapsulates the intrinsic optical geometry of the star tracker imaging system.
 *
 * Geometric Pinhole Model:
 * A 3D star ray v = [vx, vy, vz]^T expressed in the camera frame (+Z along boresight) is projected
 * onto the focal plane using perspective division:
 *   xn = vx / vz
 *   yn = vy / vz
 * Accounting for lens distortion:
 *   (xd, yd) = DistortionModel::apply(xn, yn)
 * Physical pixel coordinates:
 *   x_pixel = cx + f * xd
 *   y_pixel = cy + f * yd
 * (Delabie et al., 2013, "AIM", Eq. 6).
 *
 * Inverse Projection (Pixel to Unit Ray):
 * Given detected subpixel centroid (x, y):
 *   xd = (x - cx) / f
 *   yd = (y - cy) / f
 *   (xn, yn) = DistortionModel::remove(xd, yd)
 *   ray = [xn, yn, 1.0]^T / ||[xn, yn, 1.0]^T||
 *
 * Field-of-View (FOV) Bounds:
 * The field-of-view angle gamma determines the sky coverage of each frame:
 *   gamma = 2 * arctan(0.5 * sensor_size / f)
 * Delabie et al. (2013) demonstrate that a 20° x 20° FOV ensures that at least 3 bright stars
 * (magnitude <= 5.3) are visible across 100% of the celestial sphere.
 *
 * @reference
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." JGCD, 36(6), pp. 1672–1680.
 *   Section II.A "Pinhole Model and Camera Distortions", Eq. (6). DOI: 10.2514/1.61082.
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." JGCD, 36(2), pp. 476–488. DOI: 10.2514/1.56860.
 */

#pragma once

#include "camera/distortion.hpp"
#include "math/vector.hpp"
#include <cmath>

namespace star_tracker {

/**
 * @struct CameraModel
 * @brief Intrinsic calibration and geometric projection model for a star tracker camera.
 */
struct CameraModel {
    int width{1024};                 ///< Sensor horizontal resolution in pixels
    int height{1024};                ///< Sensor vertical resolution in pixels

    double focal_length{1000.0};     ///< Effective focal length in pixels (F)
    double principal_x{512.0};       ///< Principal point X (intersection of optical axis and sensor plane)
    double principal_y{512.0};       ///< Principal point Y (intersection of optical axis and sensor plane)

    double horizontal_fov{0.34906585}; ///< Horizontal field-of-view in radians (20 degrees)
    double vertical_fov{0.34906585};   ///< Vertical field-of-view in radians (20 degrees)

    DistortionModel distortion;      ///< 5th-order lens radial distortion calibration

    /// Default constructor initializing to standard 1024x1024, 20° FOV star tracker camera.
    CameraModel() = default;

    /**
     * @brief Full parameter constructor.
     * @param w Sensor width (pixels).
     * @param h Sensor height (pixels).
     * @param f Focal length (pixels).
     * @param cx Principal point X coordinate.
     * @param cy Principal point Y coordinate.
     * @param hfov Horizontal FOV (radians).
     * @param vfov Vertical FOV (radians).
     * @param dist Lens distortion model.
     */
    CameraModel(int w, int h, double f, double cx, double cy, double hfov, double vfov,
                DistortionModel dist = DistortionModel())
        : width(w), height(h), focal_length(f), principal_x(cx), principal_y(cy),
          horizontal_fov(hfov), vertical_fov(vfov), distortion(dist) {}

    /**
     * @brief Constructs camera model directly from sensor resolution and angular field-of-view.
     * @details
     * Computes focal length f = (0.5 * width) / tan(0.5 * hfov) and sets principal point at sensor center.
     * @param width Image width in pixels.
     * @param height Image height in pixels.
     * @param hfov_rad Horizontal field of view in radians.
     * @param vfov_rad Vertical field of view in radians.
     * @param dist Lens distortion model (default: distortion-free).
     */
    static CameraModel from_fov(int width, int height, double hfov_rad, double vfov_rad,
                                DistortionModel dist = DistortionModel()) {
        double fx = (width * 0.5) / std::tan(hfov_rad * 0.5);
        double fy = (height * 0.5) / std::tan(vfov_rad * 0.5);
        double f = 0.5 * (fx + fy);
        double cx = width * 0.5;
        double cy = height * 0.5;
        return CameraModel(width, height, f, cx, cy, hfov_rad, vfov_rad, dist);
    }

    /**
     * @brief Projects a 3D unit ray in camera frame (+Z forward) to 2D pixel coordinates (x, y).
     * @details
     * Evaluates perspective projection followed by radial lens distortion:
     *   xn = ray.x / ray.y, yn = ray.y / ray.z
     *   (xd, yd) = distortion(xn, yn)
     *   x = cx + f * xd, y = cy + f * yd
     *
     * @param ray_cam 3D vector in camera frame.
     * @param[out] x Resulting focal plane pixel X coordinate.
     * @param[out] y Resulting focal plane pixel Y coordinate.
     * @return true if ray points into forward hemisphere (ray.z > 0), false otherwise.
     */
    bool vector_to_pixel(const Vector3& ray_cam, double& x, double& y) const {
        if (ray_cam.z <= 1e-6) {
            return false;
        }
        double xn = ray_cam.x / ray_cam.z;
        double yn = ray_cam.y / ray_cam.z;

        double xd = xn, yd = yn;
        distortion.apply(xn, yn, xd, yd);

        x = principal_x + focal_length * xd;
        y = principal_y + focal_length * yd;
        return true;
    }

    /**
     * @brief Projects 2D focal-plane pixel coordinates (x, y) back into a normalized 3D ray in camera frame.
     * @details
     * Inverts sensor offset, focal scaling, and lens distortion:
     *   xd = (x - cx) / f, yd = (y - cy) / f
     *   (xn, yn) = remove_distortion(xd, yd)
     *   ray = [xn, yn, 1.0]^T / sqrt(xn^2 + yn^2 + 1)
     *
     * @param x Measured pixel X coordinate.
     * @param y Measured pixel Y coordinate.
     * @return Normalized 3D line-of-sight unit ray.
     */
    Vector3 pixel_to_vector(double x, double y) const {
        double xd = (x - principal_x) / focal_length;
        double yd = (y - principal_y) / focal_length;

        double xn = xd, yn = yd;
        distortion.remove(xd, yd, xn, yn);

        return Vector3(xn, yn, 1.0).normalized();
    }

    /**
     * @brief Checks if a pixel coordinate lies within sensor bounds, with optional edge margin.
     * @param x Pixel X coordinate.
     * @param y Pixel Y coordinate.
     * @param margin Guard margin in pixels from sensor boundary.
     * @return true if within active pixel area.
     */
    bool is_in_bounds(double x, double y, double margin = 0.0) const {
        return (x >= margin && x < (width - margin) &&
                y >= margin && y < (height - margin));
    }

    /**
     * @brief Checks if a 3D line-of-sight ray falls within the camera's active field-of-view.
     * @param ray_cam 3D vector in camera coordinates.
     * @param margin Edge guard margin in pixels.
     * @return true if ray is in FOV.
     */
    bool is_in_fov(const Vector3& ray_cam, double margin = 0.0) const {
        double x = 0.0, y = 0.0;
        if (!vector_to_pixel(ray_cam, x, y)) {
            return false;
        }
        return is_in_bounds(x, y, margin);
    }
};

} // namespace star_tracker
