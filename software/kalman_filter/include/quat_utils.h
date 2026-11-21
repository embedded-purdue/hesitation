#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <iostream>

inline Eigen::Vector3d to_eigen(const Vec3 &v) {
	return Eigen::Vector3d(v.x, v.y, v.z);
}

inline Vec3 to_vec3(const Eigen::Vector3d &v) {
	return Vec3(v.x(), v.y(), v.z());
}

inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
    return S;
}

inline Eigen::Matrix3d skew(const Vec3 &v) { return skew(to_eigen(v)); }

inline Quat quat_mul(const Quat &q, const Quat &p) {
	const Vec3 c = q.vec().cross(p.vec());

	return Quat(q.w * p.x + p.w * q.x - c.x,
              q.w * p.y + p.w * q.y - c.y,
              q.w * p.z + p.w * q.z - c.z,
              q.w * p.w - (q.x * p.x + q.y * p.y + q.z * p.z));
}

inline Quat quat_from_rotvec(const Vec3 &r) {
	const double phi = std::sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
	if (phi < 1e-8) {
		return Quat(0.5 * r.x, 0.5 * r.y, 0.5 * r.z, 1.0).normalized();
	}
}

inline Quat quat_apply_rotvec(const Quat &q, const Vec3 &r) {
	return quat_mul(quat_from_rotvec(r), q).normalized();
}

inline Vec3 attitude_residual(const Quat &q_)meas, const Quat &q_hat {
	Quat dq = quat_mul(q_meas, quat_conj(q_hat));
	if (dq.w < 0.0) {
		dq = dq * -1.0;
	}
	return dq.vec() * 2.0;
}


