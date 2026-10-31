# Dual-Mode Star Tracker Software

A high-performance C++17 star-tracker software system implementing both lost-in-space star identification and attitude estimation using optimal image matching (AIM).

## Reference Papers

1. **Delabie, T., Durt, T., and Vandersteen, J.** (2013). *"Highly Robust Lost-in-Space Algorithm Based on the Shortest Distance Transform."* *Journal of Guidance, Control, and Dynamics*, 36(2), pp. 476–488. DOI: [10.2514/1.56860](https://doi.org/10.2514/1.56860).
2. **Delabie, T., De Schutter, J., and Vandenbussche, B.** (2013). *"Highly Efficient Attitude-Estimation Algorithm for Star Trackers Using Optimal Image Matching."* *Journal of Guidance, Control, and Dynamics*, 36(6), pp. 1672–1680. DOI: [10.2514/1.61082](https://doi.org/10.2514/1.61082).
3. **Maurer, C. R., Qi, R., and Raghavan, V.** (2003). *"A Linear Time Algorithm for Computing Exact Euclidean Distance Transforms of Binary Images in Arbitrary Dimensions."* *IEEE Transactions on Pattern Analysis and Machine Intelligence*, 25(2), pp. 265–270. DOI: [10.1109/TPAMI.2003.1177156](https://doi.org/10.1109/TPAMI.2003.1177156).
4. **Shuster, M. D. and Oh, S. D.** (1981). *"Three-Axis Attitude Determination from Vector Observations."* *Journal of Guidance and Control*, 4(1), pp. 70–77. DOI: [10.2514/3.19717](https://doi.org/10.2514/3.19717).

---

## System Architecture

```text
                                 [ Camera Image / Star Spots ]
                                                |
                                                v
                                       [ Star Detection ]
                                                |
                                                v
                                      [ Centroid Extraction ]
                                                |
                                                v
                                    +-----------------------+
                                    | Operating Mode Select |
                                    +-----------+-----------+
                                                |
                            +-------------------+-------------------+
                            |                                       |
                            v                                       v
               [ ACQUISITION / RECOVERY ]                      [ TRACKING ]
                            |                                       |
                            v                                       v
                Shortest Distance Transform                     AIM Optimal
                & Centroid Feature Matching                    Image Matching
                            |                                       |
                            v                                       v
                Attitude Solution (QUEST)                      Differential
                            |                               Attitude Correction
                            +-------------------+-------------------+
                                                |
                                                v
                                    [ Validated Attitude (q) ]
```

### Operating Modes

| State | Algorithm | Responsibility |
|---|---|---|
| **ACQUISITION** | Lost-in-Space (SDT + Centroid Matching + QUEST) | Autonomous star pattern identification without prior attitude knowledge |
| **TRACKING** | AIM (*Attitude estimation using Image Matching*) | Closed-form optimal 2D focal-plane attitude refinement (~2.4 µs) |
| **RECOVERY** | Lost-in-Space | Rapid reacquisition after tracking loss or large spacecraft slew |

---

## Modular Subsystems

The codebase is organized into cleanly decoupled, self-contained modules:

1. **Math Module** (`include/math/`):
   - [`vector.hpp`](file:///home/eroxyi/Desktop/Projects/hesitation/software/star_tracking/include/math/vector.hpp): 3D Euclidean vector arithmetic, cross/dot products, norms, subtended angle calculation.
   - [`matrix.hpp`](file:///home/eroxyi/Desktop/Projects/hesitation/software/star_tracking/include/math/matrix.hpp): 3x3 matrices, determinants, analytical inversion, elementary rotations, Rodrigues formula.
   - [`quaternion.hpp`](file:///home/eroxyi/Desktop/Projects/hesitation/software/star_tracking/include/math/quaternion.hpp): Hamilton unit quaternions, DCM conversion, Euler angles, small-angle rotation vectors, geodesic angular distance.
   - [`wahba.hpp`](file:///home/eroxyi/Desktop/Projects/hesitation/software/star_tracking/include/math/wahba.hpp): Shuster's QUEST algorithm solving Wahba's problem in closed form.

2. **Camera & Optical Calibration** (`include/camera/`):
   - [`camera_model.hpp`](file:///home/eroxyi/Desktop/Projects/hesitation/software/star_tracking/include/camera/camera_model.hpp): Pinhole projection, focal length, principal point, and FOV boundary checking.
   - [`distortion.hpp`](file:///home/eroxyi/Desktop/Projects/hesitation/software/star_tracking/include/camera/distortion.hpp): 5th-order polynomial radial distortion model with forward evaluation and fixed-point inversion.

3. **Image Detection & Centroiding** (`include/detection/`):
   - [`image.hpp`](file:///home/eroxyi/Desktop/Projects/hesitation/software/star_tracking/include/detection/image.hpp): Monochromatic 8-bit image buffer, PGM I/O, Gaussian PSF synthetic star field rendering.
   - [`centroid_extractor.hpp`](file:///home/eroxyi/Desktop/Projects/hesitation/software/star_tracking/include/detection/centroid_extractor.hpp): Subpixel brightness-weighted Center-of-Gravity (CoG) and flux integration.
   - [`star_detector.hpp`](file:///home/eroxyi/Desktop/Projects/hesitation/software/star_tracking/include/detection/star_detector.hpp): Connected component labeling (8-connectivity BFS), adaptive thresholding ($\mu + k\sigma$), and region filtering.

4. **Star Catalog** (`include/catalog/`):
   - [`star.hpp`](file:///home/eroxyi/Desktop/Projects/hesitation/software/star_tracking/include/catalog/star.hpp): Celestial coordinates (RA, Dec, magnitude) and unit direction vectors.
   - [`star_catalog.hpp`](file:///home/eroxyi/Desktop/Projects/hesitation/software/star_tracking/include/catalog/star_catalog.hpp): CSV catalog parser, magnitude filtering, perspective FOV projection.

5. **Reference Database** (`include/database/`):
   - [`reference_image.hpp`](file:///home/eroxyi/Desktop/Projects/hesitation/software/star_tracking/include/database/reference_image.hpp): Reference image records and precomputed Delabie 3-star centroid features.
   - [`reference_database.hpp`](file:///home/eroxyi/Desktop/Projects/hesitation/software/star_tracking/include/database/reference_database.hpp): Compact binary serialization (`STRKDB01`), metadata JSON export, compatibility verification.
   - [`database_generator.hpp`](file:///home/eroxyi/Desktop/Projects/hesitation/software/star_tracking/include/database/database_generator.hpp): Fibonacci spiral covering sphere generator with precomputed feature combinations.

6. **Lost-in-Space (SDT)** (`include/lost_in_space/`):
   - [`distance_transform.hpp`](file:///home/eroxyi/Desktop/Projects/hesitation/software/star_tracking/include/lost_in_space/distance_transform.hpp): Maurer linear-time exact squared Euclidean distance transform ($O(W \times H)$).
   - [`centroid_matcher.hpp`](file:///home/eroxyi/Desktop/Projects/hesitation/software/star_tracking/include/lost_in_space/centroid_matcher.hpp): 2D rigid transformation $(t_x, t_y, \phi)$ aligning reference and camera centroids.
   - [`candidate_filter.hpp`](file:///home/eroxyi/Desktop/Projects/hesitation/software/star_tracking/include/lost_in_space/candidate_filter.hpp): 6-feature geometric candidate rejection ($d_A, d_B, d_C, \alpha_{\min}, \alpha_{AC}, \alpha_{BC}$) pruning $>90\%$ of candidates.
   - [`lost_in_space_matcher.hpp`](file:///home/eroxyi/Desktop/Projects/hesitation/software/star_tracking/include/lost_in_space/lost_in_space_matcher.hpp): Dual criterion evaluation (close star count and percentage) and QUEST attitude determination.

7. **Optimal Image Matching (AIM)** (`include/aim/`):
   - [`aim_estimator.hpp`](file:///home/eroxyi/Desktop/Projects/hesitation/software/star_tracking/include/aim/aim_estimator.hpp): Analytical closed-form focal-plane attitude refinement ($\sim 2.4\ \mu\text{s}$).

8. **Attitude State Machine** (`include/tracker/`):
   - [`attitude_estimate.hpp`](file:///home/eroxyi/Desktop/Projects/hesitation/software/star_tracking/include/tracker/attitude_estimate.hpp): Solution quaternion, validity flag, confidence metric, and residual diagnostics.
   - [`attitude_manager.hpp`](file:///home/eroxyi/Desktop/Projects/hesitation/software/star_tracking/include/tracker/attitude_manager.hpp): Finite state machine orchestrating ACQUISITION, TRACKING, and RECOVERY.

---

## Directory Structure

```text
star_tracking/
├── CMakeLists.txt
├── README.md
├── .gitignore
├── optimal_image_matching_AIM.txt    # Reference paper: Delabie et al. 2013 (AIM)
├── shortest_distance_transform.txt   # Reference paper: Delabie et al. 2013 (SDT)
├── include/
│   ├── star_tracker.hpp              # Master umbrella header
│   ├── camera/
│   │   ├── camera_model.hpp
│   │   └── distortion.hpp
│   ├── detection/
│   │   ├── image.hpp
│   │   ├── star_detector.hpp
│   │   └── centroid_extractor.hpp
│   ├── catalog/
│   │   ├── star.hpp
│   │   └── star_catalog.hpp
│   ├── database/
│   │   ├── reference_image.hpp
│   │   ├── reference_database.hpp
│   │   └── database_generator.hpp
│   ├── lost_in_space/
│   │   ├── distance_transform.hpp
│   │   ├── centroid_matcher.hpp
│   │   ├── candidate_filter.hpp
│   │   └── lost_in_space_matcher.hpp
│   ├── aim/
│   │   └── aim_estimator.hpp
│   ├── math/
│   │   ├── vector.hpp
│   │   ├── matrix.hpp
│   │   ├── quaternion.hpp
│   │   └── wahba.hpp
│   └── tracker/
│       ├── attitude_estimate.hpp
│       └── attitude_manager.hpp
├── src/
│   ├── main.cpp
│   ├── aim/
│   ├── camera/
│   ├── catalog/
│   ├── database/
│   ├── detection/
│   ├── lost_in_space/
│   └── tracker/
│       └── attitude_manager.cpp
├── tools/
│   └── generate_database.cpp
├── data/
│   ├── catalog/
│   │   └── hipparcos.csv
│   └── generated/
│       ├── reference_database.bin
│       └── database_metadata.json
└── tests/
    ├── test_camera_model.cpp
    ├── test_centroid_extractor.cpp
    ├── test_distance_transform.cpp
    ├── test_centroid_matcher.cpp
    ├── test_lost_in_space.cpp
    ├── test_aim.cpp
    ├── test_quaternion.cpp
    └── test_attitude_manager.cpp
```

---

## Build and Testing

### Prerequisites

- Modern C++17 compiler (GCC 9+, Clang 10+, MSVC 2019+)
- CMake 3.16+
- Make or Ninja

### Build Instructions

```bash
mkdir -p build
cd build
cmake ..
cmake --build . -j$(nproc)
```

### Running Unit Tests

```bash
cd build
ctest --output-on-failure
```

All 8 test suites cover:
- Camera model projection and distortion round-trips
- Subpixel centroid extraction accuracy
- Exact Euclidean distance transform against brute-force (100% pixel matching)
- Centroid method feature alignment and candidate filtering
- Full Lost-in-Space identification under false star noise
- AIM closed-form translation and rotation parameter recovery
- Attitude state machine transitions: Acquisition $\rightarrow$ Tracking $\rightarrow$ Recovery

---

## Running the Applications

### 1. Offline Database Generation

```bash
./build/generate_database data/catalog/hipparcos.csv data/generated/reference_database.bin data/generated/database_metadata.json
```

### 2. End-to-End Simulation Demonstration

```bash
./build/star_tracker_main
```

### Benchmark Results (1024x1024, 20° FOV)

| Pipeline Stage | Execution Time | Accuracy / Confidence |
|---|---|---|
| **Lost-in-Space Acquisition** | **~3.9 ms** | **0.0 arcsec** (exact QUEST solution) |
| **AIM Tracking Refinement** | **~2.4 µs** | **< 0.7 arcsec** attitude error |
| **Tracking Failure Recovery** | **~2.8 ms** | **0.0 arcsec** re-acquisition |

---

## Mathematical Highlights

### Maurer Shortest Distance Transform (Delabie et al. 2013 SDT, Maurer et al. 2003)

Computes exact squared Euclidean distance $D(x, y) = \min_{s} [(x - s_x)^2 + (y - s_y)^2]$ in $O(W \times H)$ time via 1D dimensional separation and parabolic envelope construction.

### Delabie Centroid Method (Delabie et al. 2013 SDT)

Aligns camera star patterns to reference images using bright-star triplets:
- Centroid $C = \frac{1}{3} \sum_{i=1}^3 S_i$
- 6 geometric invariant features: 3 distances from centroid $(d_A, d_B, d_C)$ and 3 pairwise angles $(\alpha_{\min}, \alpha_{AC}, \alpha_{BC})$
- Canonical star ordering keyed on the minimum pairwise angle $\alpha_{\min}$
- Rejects >90% of database candidates prior to querying the distance map

### AIM Closed-Form Minimization (Delabie et al. 2013 AIM)

Minimizes the focal-plane coordinate cost:
$$\Gamma(\phi, t_x, t_y) = \sum_{i=1}^{n_s} w_i \left[ (x_i - x'_i)^2 + (y_i - y'_i)^2 \right]$$
with closed-form optimal parameters:
$$\phi = \text{atan2}(S, C)$$
where $C = \sum w_i (\Delta x_i \Delta \hat{x}_i + \Delta y_i \Delta \hat{y}_i)$ and $S = \sum w_i (\Delta x_i \Delta \hat{y}_i - \Delta y_i \Delta \hat{x}_i)$.
Rotational correction:
$$\mathbf{q}_e = \mathbf{q}_{\text{diff}} \otimes \mathbf{q}_{\text{dat}}$$
where $\boldsymbol{\delta} = (-t_v / f, \; t_u / f, \; -\phi)^T$ relative to the camera optical axis.
