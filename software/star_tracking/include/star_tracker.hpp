/**
 * @file star_tracker.hpp
 * @brief Master umbrella include header for the Dual-Mode Star Tracker software system.
 *
 * @details
 * Including this single header provides full access to the complete star tracker library:
 * mathematical primitives, camera models, image detection, reference databases,
 * Lost-in-Space algorithms (Maurer SDT + Delabie Centroid Method), AIM tracking,
 * and the top-level AttitudeManager module.
 *
 * Usage:
 * @code
 *   #include "star_tracker.hpp"
 *   using namespace star_tracker;
 *
 *   CameraModel camera = CameraModel::from_fov(1024, 1024, 20.0 * M_PI / 180.0, 20.0 * M_PI / 180.0);
 *   StarCatalog catalog;
 *   catalog.load_from_csv("data/catalog/hipparcos.csv");
 *   ReferenceDatabase db;
 *   db.load_binary("data/generated/reference_database.bin");
 *
 *   AttitudeManager manager(camera, db, catalog);
 *   AttitudeEstimate estimate = manager.process_frame(image);
 * @endcode
 *
 * @reference
 * - Delabie, T., Durt, T., and Vandersteen, J. (2013). "Highly Robust Lost-in-Space Algorithm
 *   Based on the Shortest Distance Transform." Journal of Guidance, Control, and Dynamics, 36(2),
 *   pp. 476–488. DOI: 10.2514/1.56860.
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." Journal of Guidance, Control,
 *   and Dynamics, 36(6), pp. 1672–1680. DOI: 10.2514/1.61082.
 * - Maurer, C. R., Qi, R., and Raghavan, V. (2003). "A Linear Time Algorithm for Computing
 *   Exact Euclidean Distance Transforms of Binary Images in Arbitrary Dimensions." IEEE TPAMI,
 *   25(2), pp. 265–270.
 */

#pragma once

// Mathematical Foundations
#include "math/vector.hpp"
#include "math/matrix.hpp"
#include "math/quaternion.hpp"
#include "math/wahba.hpp"

// Optics and Camera Calibration
#include "camera/distortion.hpp"
#include "camera/camera_model.hpp"

// Image Buffer and Star Spot Detection
#include "detection/image.hpp"
#include "detection/centroid_extractor.hpp"
#include "detection/star_detector.hpp"

// Celestial Star Catalog
#include "catalog/star.hpp"
#include "catalog/star_catalog.hpp"

// Reference Database and Precomputed Features
#include "database/reference_image.hpp"
#include "database/reference_database.hpp"
#include "database/database_generator.hpp"

// Lost-in-Space (SDT + Centroid Matching)
#include "lost_in_space/distance_transform.hpp"
#include "lost_in_space/centroid_matcher.hpp"
#include "lost_in_space/candidate_filter.hpp"
#include "lost_in_space/lost_in_space_matcher.hpp"

// Closed-Form Attitude Estimation using Image Matching (AIM)
#include "aim/aim_estimator.hpp"

// Attitude Manager State Machine
#include "tracker/attitude_estimate.hpp"
#include "tracker/attitude_manager.hpp"
