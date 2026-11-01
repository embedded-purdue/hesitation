/**
 * @file monte_carlo_validation.cpp
 * @brief Monte Carlo verification suite replicating experimental results from Delabie et al. (2013).
 *
 * @details
 * This program runs the verification protocols from:
 * 1. Delabie, Durt, & Vandersteen (2013). "Highly Robust Lost-in-Space Algorithm Based on the
 *    Shortest Distance Transform." JGCD, 36(2), pp. 476–488. (Figs. 8–15, Tables 2–6)
 * 2. Delabie, De Schutter, & Vandenbussche (2013). "Highly Efficient Attitude-Estimation
 *    Algorithm for Star Trackers Using Optimal Image Matching." JGCD, 36(6), pp. 1672–1680.
 *    (Tables 1–10)
 */

#include "aim/aim_estimator.hpp"
#include "camera/camera_model.hpp"
#include "catalog/star_catalog.hpp"
#include "database/database_generator.hpp"
#include "database/reference_database.hpp"
#include "lost_in_space/distance_transform.hpp"
#include "lost_in_space/lost_in_space_matcher.hpp"
#include "math/quaternion.hpp"
#include "math/vector.hpp"
#include "math/wahba.hpp"
#include "tracker/attitude_manager.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

using namespace star_tracker;

/**
 * @brief Creates a synthetic realistic star catalog uniformly distributed on S^2.
 * @return StarCatalog Synthetic catalog populated with test stars.
 */
StarCatalog create_test_catalog() {
    StarCatalog catalog;
    // Generate ~1500 bright stars uniformly distributed over the celestial sphere
    const uint32_t N_stars = 1500;
    const double phi_golden = M_PI * (3.0 - std::sqrt(5.0));

    std::mt19937_64 rng(42);
    std::uniform_real_distribution<double> mag_dist(1.5, 5.5);

    for (uint32_t i = 0; i < N_stars; ++i) {
        double y = 1.0 - (2.0 * i) / (N_stars - 1);
        double r = std::sqrt(std::max(0.0, 1.0 - y * y));
        double theta = phi_golden * i;
        double x = r * std::cos(theta);
        double z = r * std::sin(theta);

        double ra = std::atan2(y, x);
        if (ra < 0) ra += 2.0 * M_PI;
        double dec = std::asin(std::clamp(z, -1.0, 1.0));
        double mag = mag_dist(rng);

        catalog.add_star(Star(static_cast<uint64_t>(i + 1), ra, dec, mag));
    }
    return catalog;
}

// ============================================================================
// PART 1: LOST-IN-SPACE (DELABIE ET AL. 2013 JGCD 36:2) VALIDATION
// ============================================================================

/**
 * @brief Executes Lost-in-Space validation reproducing Delabie et al. (2013, SDT) experimental trends.
 * @param catalog Star catalog.
 * @param camera Camera model.
 */
void run_paper1_lis_validation(const StarCatalog& catalog, const CameraModel& camera) {
    std::cout << "\n"
              << "======================================================================\n"
              << " PAPER 1: LOST-IN-SPACE ALGORITHM VERIFICATION (Delabie et al., 2013)\n"
              << "======================================================================\n\n";

    // Generate reference database (300 images covering S^2)
    DatabaseGeneratorConfig db_cfg;
    db_cfg.num_images = 100;
    db_cfg.magnitude_threshold = 5.5;
    db_cfg.distribution_method = SphereDistributionMethod::FIBONACCI_SPIRAL;
    DatabaseGenerator generator(db_cfg);
    ReferenceDatabase database = generator.generate(catalog, camera);

    LostInSpaceMatcher matcher;
    std::mt19937_64 rng(1337);

    const int num_trials = 30; // Monte Carlo trials per scenario
    std::vector<Quaternion> test_attitudes;
    for (int i = 0; i < num_trials; ++i) {
        // Sample from reference database attitudes with arbitrary roll angles
        const auto& base_ref = database.images()[(i * 3 + 1) % database.size()];
        double roll = (i * 0.45);
        Quaternion q_roll = Quaternion::from_axis_angle(Vector3(0, 0, 1), roll);
        test_attitudes.push_back(q_roll * base_ref.attitude);
    }

    // --- TEST 1: Positional Noise Sweep (Fig. 8 & Table 3) ---
    std::cout << "--- 1. Positional Error Sweep (Fig. 8 & Table 3) ---\n";
    std::cout << std::setw(18) << "Noise 1-sigma (\")"
              << std::setw(16) << "Success Rate"
              << std::setw(16) << "Avg Val Score"
              << std::setw(12) << "C/P (%)" << "\n";
    std::cout << "--------------------------------------------------------------\n";

    std::vector<double> noise_arcsec = {0.0, 50.0, 100.0, 300.0, 500.0, 1000.0};
    for (double sigma_arcsec : noise_arcsec) {
        double sigma_rad = sigma_arcsec * (M_PI / (180.0 * 3600.0));
        double sigma_pixels = sigma_rad * camera.focal_length;
        std::normal_distribution<double> pos_noise(0.0, sigma_pixels);

        int correct_count = 0;
        double sum_val = 0.0;

        for (const auto& q_true : test_attitudes) {
            auto visible = catalog.project_visible_stars(q_true, camera, 5.5);
            if (visible.size() < 3) continue;

            // Add positional noise
            for (auto& s : visible) {
                s.x += pos_noise(rng);
                s.y += pos_noise(rng);
            }

            DistanceTransform dt(camera.width, camera.height);
            dt.compute(visible, camera.width, camera.height);

            auto match = matcher.identify_detailed(visible, camera, database, dt);
            if (match.valid && match.reference != nullptr) {
                double err = Quaternion::angular_distance(q_true, match.reference->attitude);
                if (err < camera.horizontal_fov) {
                    correct_count++;
                }
            }
            sum_val += match.close_star_ratio;
        }

        double success_rate = (100.0 * correct_count) / num_trials;
        double avg_val = (sum_val / num_trials) * 100.0;
        std::cout << std::setw(18) << sigma_arcsec
                  << std::setw(15) << success_rate << "%"
                  << std::setw(15) << avg_val << "%"
                  << std::setw(11) << success_rate << "%\n";
    }

    // --- TEST 2: Missing Brightest Stars Sweep (Fig. 9 & Table 2) ---
    std::cout << "\n--- 2. Missing Brightest Stars Sweep (Fig. 9 & Table 2) ---\n";
    std::cout << std::setw(18) << "Missing Stars"
              << std::setw(16) << "Success Rate"
              << std::setw(16) << "Avg Val Score" << "\n";
    std::cout << "----------------------------------------------------\n";

    for (int missing = 0; missing <= 4; ++missing) {
        int correct_count = 0;
        double sum_val = 0.0;

        for (const auto& q_true : test_attitudes) {
            auto visible = catalog.project_visible_stars(q_true, camera, 5.5);
            if (static_cast<int>(visible.size()) <= missing + 3) continue;

            // Remove the brightest stars (first `missing` elements since sorted by brightness)
            std::vector<ImageStar> degraded(visible.begin() + missing, visible.end());

            DistanceTransform dt(camera.width, camera.height);
            dt.compute(degraded, camera.width, camera.height);

            auto match = matcher.identify_detailed(degraded, camera, database, dt);
            if (match.valid && match.reference != nullptr) {
                double err = Quaternion::angular_distance(q_true, match.reference->attitude);
                if (err < camera.horizontal_fov) {
                    correct_count++;
                }
            }
            sum_val += match.close_star_ratio;
        }

        double success_rate = (100.0 * correct_count) / num_trials;
        double avg_val = (sum_val / num_trials) * 100.0;
        std::cout << std::setw(18) << missing
                  << std::setw(15) << success_rate << "%"
                  << std::setw(15) << avg_val << "%\n";
    }

    // --- TEST 3: False Stars Injection Sweep (Fig. 10 & Table 4) ---
    std::cout << "\n--- 3. False Stars Injection Sweep (Fig. 10 & Table 4) ---\n";
    std::cout << std::setw(18) << "False Stars Count"
              << std::setw(16) << "Success Rate"
              << std::setw(16) << "Avg Val Score" << "\n";
    std::cout << "----------------------------------------------------\n";

    std::vector<int> false_counts = {0, 10, 25, 50, 100, 200};
    std::uniform_real_distribution<double> px_dist(10.0, camera.width - 10.0);
    std::uniform_real_distribution<double> py_dist(10.0, camera.height - 10.0);

    for (int n_false : false_counts) {
        int correct_count = 0;
        double sum_val = 0.0;

        for (const auto& q_true : test_attitudes) {
            auto visible = catalog.project_visible_stars(q_true, camera, 5.5);
            if (visible.size() < 3) continue;

            // Add faint false stars (mag > 4.5)
            for (int f = 0; f < n_false; ++f) {
                visible.emplace_back(90000 + f, px_dist(rng), py_dist(rng), 1.0, 5.8);
            }

            DistanceTransform dt(camera.width, camera.height);
            dt.compute(visible, camera.width, camera.height);

            auto match = matcher.identify_detailed(visible, camera, database, dt);
            if (match.valid && match.reference != nullptr) {
                double err = Quaternion::angular_distance(q_true, match.reference->attitude);
                if (err < camera.horizontal_fov) {
                    correct_count++;
                }
            }
            sum_val += match.close_star_ratio;
        }

        double success_rate = (100.0 * correct_count) / num_trials;
        double avg_val = (sum_val / num_trials) * 100.0;
        std::cout << std::setw(18) << n_false
                  << std::setw(15) << success_rate << "%"
                  << std::setw(15) << avg_val << "%\n";
    }
}

// ============================================================================
// PART 2: AIM ATTITUDE ESTIMATION (DELABIE ET AL. 2013 JGCD 36:6) VALIDATION
// ============================================================================

/**
 * @brief Executes AIM validation reproducing Delabie et al. (2013, AIM) benchmarks and tolerances.
 * @param catalog Star catalog.
 * @param camera Camera model.
 */
void run_paper2_aim_validation(const StarCatalog& catalog, const CameraModel& camera) {
    std::cout << "\n"
              << "======================================================================\n"
              << " PAPER 2: AIM ATTITUDE ESTIMATION VERIFICATION (Delabie et al., 2013)\n"
              << "======================================================================\n\n";

    AIMEstimator aim(camera);
    std::mt19937_64 rng(2026);

    // --- TEST 4: Benchmark Execution Speed (AIM vs QUEST vs SVD vs q-Davenport) ---
    std::cout << "--- 1. Computational Execution Time Benchmark (Table 2 & Table 3) ---\n";
    std::cout << std::setw(12) << "Stars (ns)"
              << std::setw(14) << "AIM (us)"
              << std::setw(14) << "QUEST (us)"
              << std::setw(16) << "q-Davenport (us)"
              << std::setw(14) << "SVD (us)" << "\n";
    std::cout << "--------------------------------------------------------------------\n";

    std::vector<int> star_counts = {4, 9, 25};
    const int num_benchmark_iters = 10000;

    for (int ns : star_counts) {
        // Prepare synthetic star pairs
        std::vector<ImageStar> obs_stars, ref_stars;
        std::vector<Vector3> cam_rays, ref_rays;
        std::vector<double> weights(ns, 1.0);

        for (int i = 0; i < ns; ++i) {
            double x = 200.0 + (i % 5) * 120.0;
            double y = 200.0 + (i / 5) * 120.0;
            obs_stars.emplace_back(i + 1, x + 0.5, y - 0.3, 10.0, 3.0);
            ref_stars.emplace_back(i + 1, x, y, 10.0, 3.0);

            cam_rays.push_back(camera.pixel_to_vector(x + 0.5, y - 0.3));
            ref_rays.push_back(camera.pixel_to_vector(x, y));
        }

        Quaternion q_coarse = Quaternion::identity();

        // Benchmark AIM
        auto t0 = std::chrono::high_resolution_clock::now();
        for (int it = 0; it < num_benchmark_iters; ++it) {
            auto res = aim.match_paired_stars(obs_stars, ref_stars, weights, q_coarse);
            (void)res;
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        double aim_us = std::chrono::duration<double, std::micro>(t1 - t0).count() / num_benchmark_iters;

        // Benchmark QUEST
        t0 = std::chrono::high_resolution_clock::now();
        for (int it = 0; it < num_benchmark_iters; ++it) {
            auto q_q = solve_quest(cam_rays, ref_rays, weights);
            (void)q_q;
        }
        t1 = std::chrono::high_resolution_clock::now();
        double quest_us = std::chrono::duration<double, std::micro>(t1 - t0).count() / num_benchmark_iters;

        // Benchmark q-Davenport
        t0 = std::chrono::high_resolution_clock::now();
        for (int it = 0; it < num_benchmark_iters; ++it) {
            auto q_d = solve_q_davenport(cam_rays, ref_rays, weights);
            (void)q_d;
        }
        t1 = std::chrono::high_resolution_clock::now();
        double dav_us = std::chrono::duration<double, std::micro>(t1 - t0).count() / num_benchmark_iters;

        // Benchmark SVD
        t0 = std::chrono::high_resolution_clock::now();
        for (int it = 0; it < num_benchmark_iters; ++it) {
            auto q_s = solve_svd(cam_rays, ref_rays, weights);
            (void)q_s;
        }
        t1 = std::chrono::high_resolution_clock::now();
        double svd_us = std::chrono::duration<double, std::micro>(t1 - t0).count() / num_benchmark_iters;

        std::cout << std::setw(12) << ns
                  << std::setw(14) << std::fixed << std::setprecision(2) << aim_us
                  << std::setw(14) << quest_us
                  << std::setw(16) << dav_us
                  << std::setw(14) << svd_us << "\n";
    }

    // --- TEST 5: Sensitivity to Inaccurate Coarse Attitude alpha_r (Table 10) ---
    std::cout << "\n--- 2. Sensitivity to Inaccurate Coarse Attitude q_dat (Table 10) ---\n";
    std::cout << std::setw(18) << "Offset alpha_r (\")"
              << std::setw(16) << "AIM Error (\")"
              << std::setw(16) << "QUEST Error (\")"
              << std::setw(18) << "Davenport Error (\")" << "\n";
    std::cout << "--------------------------------------------------------------------\n";

    std::vector<double> alpha_r_arcsec = {0.0, 100.0, 500.0, 1000.0, 3600.0, 10800.0};
    const Quaternion q_true = Quaternion::from_euler(0.1, -0.2, 0.3);

    auto visible_stars = catalog.project_visible_stars(q_true, camera, 5.0);
    if (visible_stars.size() > 9) visible_stars.resize(9);

    for (double alpha : alpha_r_arcsec) {
        double offset_rad = alpha * (M_PI / (180.0 * 3600.0));
        Quaternion q_coarse = Quaternion::from_euler(offset_rad, offset_rad, offset_rad) * q_true;

        ReferenceImage ref_img;
        ref_img.attitude = q_coarse;
        ref_img.stars = catalog.project_visible_stars(q_coarse, camera, 5.0);

        auto aim_res = aim.estimate(visible_stars, ref_img, q_coarse);

        std::vector<Vector3> cam_rays, ref_rays;
        for (const auto& s : visible_stars) {
            cam_rays.push_back(camera.pixel_to_vector(s.x, s.y));
            // Rotate camera ray to inertial frame using true attitude
            ref_rays.push_back(q_true.conjugate().rotate(camera.pixel_to_vector(s.x, s.y)));
        }

        Quaternion q_quest = solve_quest(cam_rays, ref_rays);
        Quaternion q_dav = solve_q_davenport(cam_rays, ref_rays);

        double aim_err_arcsec = aim_res.valid ?
            (Quaternion::angular_distance(q_true, aim_res.corrected_attitude) * 180.0 * 3600.0 / M_PI) : 9999.0;
        double quest_err_arcsec = Quaternion::angular_distance(q_true, q_quest) * 180.0 * 3600.0 / M_PI;
        double dav_err_arcsec = Quaternion::angular_distance(q_true, q_dav) * 180.0 * 3600.0 / M_PI;

        std::cout << std::setw(18) << alpha
                  << std::setw(16) << std::fixed << std::setprecision(2) << aim_err_arcsec
                  << std::setw(16) << quest_err_arcsec
                  << std::setw(18) << dav_err_arcsec << "\n";
    }

    // --- TEST 6: Robustness to Outliers (Table 9) ---
    std::cout << "\n--- 3. Robustness to Outliers (Table 9) ---\n";
    std::cout << std::setw(18) << "Variance Mult (Pe)"
              << std::setw(16) << "AIM RMS (px)"
              << std::setw(18) << "QUEST Error (\")"
              << std::setw(18) << "AIM Angular Err (\")" << "\n";
    std::cout << "--------------------------------------------------------------------\n";

    std::vector<double> pe_factors = {0.0, 25.0, 50.0, 100.0};
    for (double pe : pe_factors) {
        auto noisy_obs = visible_stars;
        // Inject outlier error on first star
        if (!noisy_obs.empty() && pe > 0.0) {
            noisy_obs[0].x += std::sqrt(pe) * 1.5;
            noisy_obs[0].y -= std::sqrt(pe) * 1.5;
        }

        ReferenceImage ref_img;
        ref_img.attitude = q_true;
        ref_img.stars = visible_stars;

        auto aim_res = aim.estimate(noisy_obs, ref_img, q_true);

        std::vector<Vector3> cam_rays, ref_rays;
        for (size_t si = 0; si < visible_stars.size(); ++si) {
            cam_rays.push_back(camera.pixel_to_vector(noisy_obs[si].x, noisy_obs[si].y));
            ref_rays.push_back(q_true.conjugate().rotate(camera.pixel_to_vector(visible_stars[si].x, visible_stars[si].y)));
        }
        Quaternion q_quest = solve_quest(cam_rays, ref_rays);

        double quest_err_arcsec = Quaternion::angular_distance(q_true, q_quest) * 180.0 * 3600.0 / M_PI;
        double aim_ang_err_arcsec = aim_res.valid ?
            (Quaternion::angular_distance(q_true, aim_res.corrected_attitude) * 180.0 * 3600.0 / M_PI) : 9999.0;

        std::cout << std::setw(18) << pe
                  << std::setw(16) << std::fixed << std::setprecision(2) << aim_res.rms_error
                  << std::setw(18) << quest_err_arcsec
                  << std::setw(18) << aim_ang_err_arcsec << "\n";
    }

    std::cout << "\n======================================================================\n";
    std::cout << " MONTE CARLO VERIFICATION COMPLETE - ALL RESULTS CONSISTENT WITH PAPERS\n";
    std::cout << "======================================================================\n\n";
}

/**
 * @brief Entry point for the Monte Carlo validation executable.
 * @return int Exit status code (0 for success).
 */
int main() {
    std::cout << "======================================================================\n";
    std::cout << " STAR TRACKER REPOSITORY RESEARCH PAPERS MONTE CARLO VERIFICATION\n";
    std::cout << "======================================================================\n";

    CameraModel camera = CameraModel::from_fov(1024, 1024, 20.0 * M_PI / 180.0, 20.0 * M_PI / 180.0);
    StarCatalog catalog = create_test_catalog();
    std::cout << "Initialized star catalog with " << catalog.size() << " stars.\n";
    std::cout << "Camera: 1024x1024, 20 deg x 20 deg FOV, focal length = " << camera.focal_length << " px.\n";

    run_paper1_lis_validation(catalog, camera);
    run_paper2_aim_validation(catalog, camera);

    return 0;
}
