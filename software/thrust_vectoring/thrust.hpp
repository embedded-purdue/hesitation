#pragma once

namespace nep {              // Put everything below under the name "nep" (used as nep::Vec3, nep::thrust_step)

struct Vec3 { float x, y, z; };   // A 3D vector: three decimal numbers. Used for force, torque, positions

// ---- Hardware settings: filled in once, never changed while running ----
struct ThrusterConfig {
    float isp_s;              // Specific impulse in seconds: how fuel-efficient the thruster is
    float efficiency;         // Fraction (0 to 1) of electrical power that becomes thrust energy
    float thrust_max_n;       // Most thrust the hardware can ever give, in newtons
    float gimbal_max_rad;     // Furthest the gimbal can tilt either way, in radians
    float gimbal_rate_rad_s;  // Fastest the gimbal can move, in radians per second
    Vec3  position_b_m;       // Where the thruster is mounted on the spacecraft, in metres
};

// ---- What the controller asks for, every cycle ----
struct ThrustCmd {
    float thrust_n;           // Requested thrust, newtons
    float pitch_rad;          // Requested up/down tilt, radians
    float yaw_rad;            // Requested left/right tilt, radians
};

// ---- Where the gimbal actually is (remembered between cycles) ----
struct ThrusterState {
    float pitch_rad;          // Current actual up/down tilt
    float yaw_rad;            // Current actual left/right tilt
};

// ---- What actually happened this cycle ----
struct ThrustOut {
    float thrust_n;           // Thrust actually delivered after all limits, newtons
    Vec3  force_b_n;          // Push as a 3D vector (goes to the dynamics: makes the craft accelerate)
    Vec3  torque_b_nm;        // Turning force as a 3D vector (goes to the dynamics: makes the craft rotate)
    float mdot_kg_s;          // Propellant used per second, kg/s (goes to the mass model)
    float power_used_w;       // Electricity drawn, watts (goes to PMAD)
    bool  power_limited;      // true = you asked for more than the available power could give
};

// Declaration only (the body is in thrust.cpp). Runs one control cycle.
//   const X&  = read-only input, passed without copying
//   X&        = the function writes into it
void thrust_step(const ThrusterConfig& cfg,   // hardware settings (read-only)
                 ThrusterState& st,           // gimbal position (read AND updated)
                 const ThrustCmd& cmd,        // the request (read-only)
                 float p_avail_w,             // power available from PMAD, watts
                 const Vec3& com_b_m,         // current center of mass, metres (read-only)
                 float dt_s,                  // time step, seconds
                 ThrustOut& out);             // results are written here

// Declaration only. Works backward: "what tilt gives me this turning force?"
// Returns true if it worked, false if thrust or lever arm is ~0.
bool tvc_gimbal_for_torque(const ThrusterConfig& cfg,  // hardware settings
                           float thrust_n,             // thrust you plan to use
                           const Vec3& com_b_m,        // center of mass
                           float tau_y_nm,             // desired torque about Y (pitch)
                           float tau_z_nm,             // desired torque about Z (yaw)
                           ThrustCmd& cmd_out);        // gimbal angles are written here

} // namespace nep