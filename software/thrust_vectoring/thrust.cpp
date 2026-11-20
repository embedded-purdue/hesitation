//
// Created by Aidan Chan on 10/25/26.
//

#include "thrust.hpp"        // Our own header: gives this file the structs and declarations
#include <cmath>             // Math functions: std::cos, std::sin, std::fabs, std::isfinite

namespace nep {              // Same group name as the header, so these definitions match those declarations
namespace {                  // Unnamed namespace = everything inside is PRIVATE to this file

constexpr float kG0 = 9.80665f;   // Standard gravity, m/s². constexpr = fixed at compile time. f = float

// Keep v between lo and hi. "a ? b : c" means "if a then b else c".
inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);   // below lo -> lo; above hi -> hi; otherwise v
}

// true if v is a real number (not NaN, not infinity)
inline bool finite(float v) { return std::isfinite(v); }

// Cross product a × b. Used for torque = lever arm × force.
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return { a.y * b.z - a.z * b.y,     // x component
             a.z * b.x - a.x * b.z,     // y component
             a.x * b.y - a.y * b.x };   // z component
}
} // namespace               // End of the private section

// ===================== Main function: one control cycle =====================
void thrust_step(const ThrusterConfig& cfg, ThrusterState& st,
                 const ThrustCmd& cmd, float p_avail_w,
                 const Vec3& com_b_m, float dt_s, ThrustOut& out) {

    // ---- Step 1: replace garbage inputs with safe values ----
    float t_req = finite(cmd.thrust_n) ? cmd.thrust_n : 0.0f;          // bad thrust -> 0
    float p_cmd = finite(cmd.pitch_rad) ? cmd.pitch_rad : st.pitch_rad; // bad pitch -> stay where you are
    float y_cmd = finite(cmd.yaw_rad)   ? cmd.yaw_rad   : st.yaw_rad;   // bad yaw   -> stay where you are
    if (!finite(p_avail_w) || p_avail_w < 0.0f) p_avail_w = 0.0f;      // bad/negative power -> 0
                                                                       // (! = NOT, || = OR)

    // ---- Step 2: move the gimbal, respecting angle and speed limits ----
    const float gmax = cfg.gimbal_max_rad;            // max tilt angle
    const float step = cfg.gimbal_rate_rad_s * dt_s;  // max distance it can move this cycle (speed × time)
    p_cmd = clampf(p_cmd, -gmax, gmax);               // don't target beyond the angle limit
    y_cmd = clampf(y_cmd, -gmax, gmax);
    st.pitch_rad += clampf(p_cmd - st.pitch_rad, -step, step);  // move toward target, at most "step"
    st.yaw_rad   += clampf(y_cmd - st.yaw_rad,   -step, step);  // (+= adds to the stored value)

    // ---- Step 3: work out how much thrust you really get ----
    const float ve = cfg.isp_s * kG0;                           // exhaust velocity = Isp × g0
    const float t_power = 2.0f * cfg.efficiency * p_avail_w / ve; // max thrust this power allows: T = 2ηP/ve
    const float t_hw = clampf(t_req, 0.0f, cfg.thrust_max_n);   // request limited to 0..hardware max
    const float T = (t_hw < t_power) ? t_hw : t_power;          // take the smaller of the two limits
    out.power_limited = (t_power < t_hw);                       // flag: was power the bottleneck?
    out.thrust_n = T;                                           // report actual thrust

    // ---- Step 4: which way is the thrust pointing? ----
    const float cp = std::cos(st.pitch_rad), sp = std::sin(st.pitch_rad); // cos/sin of ACTUAL pitch
    const float cy = std::cos(st.yaw_rad),   sy = std::sin(st.yaw_rad);   // cos/sin of ACTUAL yaw
    const Vec3 dir = { cp * cy, sy, -sp * cy };   // +X rotated by yaw then pitch; length always 1
                                                  // (no tilt -> (1,0,0) = straight along +X)
    out.force_b_n = { T * dir.x, T * dir.y, T * dir.z };  // force = thrust × direction

    // ---- Step 5: turning force (torque) about the center of mass ----
    const Vec3 r = { cfg.position_b_m.x - com_b_m.x,      // lever arm: from center of mass
                     cfg.position_b_m.y - com_b_m.y,      //            to the thruster
                     cfg.position_b_m.z - com_b_m.z };
    out.torque_b_nm = cross(r, out.force_b_n);            // torque = r × F (zero if thrust passes through CoM)

    // ---- Step 6: propellant and power used ----
    out.mdot_kg_s    = T / ve;                            // fuel flow = thrust / exhaust velocity
    out.power_used_w = (cfg.efficiency > 0.0f)            // avoid dividing by zero
                     ? T * ve / (2.0f * cfg.efficiency)   // power = T·ve / (2η)
                     : 0.0f;
}

// ============ Reverse helper: desired torque -> gimbal angles ============
bool tvc_gimbal_for_torque(const ThrusterConfig& cfg, float thrust_n,
                           const Vec3& com_b_m, float tau_y_nm, float tau_z_nm,
                           ThrustCmd& cmd_out) {
    const float L = com_b_m.x - cfg.position_b_m.x;   // distance thruster -> CoM along X (2 m in the test)
    cmd_out.thrust_n = thrust_n;                      // pass the thrust straight through

    if (thrust_n < 1e-6f || std::fabs(L) < 1e-6f) {   // ~no thrust or ~no lever arm? (1e-6 = 0.000001)
        cmd_out.pitch_rad = 0.0f;                     //   then no tilt can make torque,
        cmd_out.yaw_rad = 0.0f;                       //   so zero the angles
        return false;                                 //   and report failure
    }

    // Small-angle physics: torque ≈ -thrust × L × angle  ->  angle = -torque / (thrust × L)
    const float gmax = cfg.gimbal_max_rad;
    cmd_out.pitch_rad = clampf(-tau_y_nm / (thrust_n * L), -gmax, gmax); // pitch tilt for Y torque, within limits
    cmd_out.yaw_rad   = clampf(-tau_z_nm / (thrust_n * L), -gmax, gmax); // yaw tilt for Z torque, within limits
    return true;                                                         // success
}

} // namespace nep