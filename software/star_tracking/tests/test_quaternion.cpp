/**
 * @file test_quaternion.cpp
 * @brief Unit tests for Hamilton unit quaternions, SO(3) rotations, and angular distance.
 *
 * @reference
 * - Delabie, T., De Schutter, J., and Vandenbussche, B. (2013). "Highly Efficient Attitude-Estimation
 *   Algorithm for Star Trackers Using Optimal Image Matching." JGCD, 36(6), pp. 1672–1680.
 *   Eqs. (5), (20–21). DOI: 10.2514/1.61082.
 */

#include <gtest/gtest.h>
#include "math/quaternion.hpp"
#include <cmath>

using namespace star_tracker;

// Verifies identity quaternion default initialization and normalization
TEST(QuaternionTest, DefaultAndIdentity) {
    Quaternion q;
    EXPECT_DOUBLE_EQ(q.w, 1.0);
    EXPECT_DOUBLE_EQ(q.x, 0.0);
    EXPECT_DOUBLE_EQ(q.y, 0.0);
    EXPECT_DOUBLE_EQ(q.z, 0.0);
    EXPECT_DOUBLE_EQ(q.norm(), 1.0);
}

// Verifies rotation of a 3D vector via axis-angle formulation
TEST(QuaternionTest, AxisAngleRotation) {
    // 90 degree rotation about Z axis: maps +X to +Y
    Vector3 z_axis(0.0, 0.0, 1.0);
    Quaternion q = Quaternion::from_axis_angle(z_axis, M_PI / 2.0);

    Vector3 v_x(1.0, 0.0, 0.0);
    Vector3 v_rot = q.rotate(v_x);

    EXPECT_NEAR(v_rot.x, 0.0, 1e-12);
    EXPECT_NEAR(v_rot.y, 1.0, 1e-12);
    EXPECT_NEAR(v_rot.z, 0.0, 1e-12);
}

// Verifies fundamental Hamilton quaternion multiplication rules: i*j = k and non-commutativity
TEST(QuaternionTest, HamiltonMultiplication) {
    Quaternion qi(0.0, 1.0, 0.0, 0.0);
    Quaternion qj(0.0, 0.0, 1.0, 0.0);
    Quaternion qk = qi * qj;

    EXPECT_NEAR(qk.w, 0.0, 1e-12);
    EXPECT_NEAR(qk.x, 0.0, 1e-12);
    EXPECT_NEAR(qk.y, 0.0, 1e-12);
    EXPECT_NEAR(qk.z, 1.0, 1e-12);

    // Non-commutative check: j*i = -k
    Quaternion qk_neg = qj * qi;
    EXPECT_NEAR(qk_neg.z, -1.0, 1e-12);
}

// Verifies round-trip conversion between Quaternion and Direction Cosine Matrix (DCM)
TEST(QuaternionTest, MatrixConversion) {
    Quaternion q = Quaternion::from_euler(0.3, -0.4, 0.5);
    Matrix3 R = q.to_matrix();
    Quaternion q_back = Quaternion::from_matrix(R);

    double dist = Quaternion::angular_distance(q, q_back);
    EXPECT_NEAR(dist, 0.0, 1e-7);
}

// Verifies spatial geodesic angular distance calculation
TEST(QuaternionTest, AngularDistance) {
    Quaternion q1 = Quaternion::identity();
    double angle = 0.05; // 50 milliradians
    Quaternion q2 = Quaternion::from_axis_angle(Vector3(0.0, 1.0, 0.0), angle);

    double measured = Quaternion::angular_distance(q1, q2);
    EXPECT_NEAR(measured, angle, 1e-10);
}
