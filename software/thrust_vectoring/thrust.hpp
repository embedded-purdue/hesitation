//
// Created by Aidan Chan on 10/25/26.
//

#ifndef HESITATION_THRUST_HPP
#define HESITATION_THRUST_HPP

#endif // HESITATION_THRUST_HPP

namespace nep {
struct Vec3 { float x, y, z; };

// Change later for hardware
struct ThrusterConfig {
    float isp_s;              // specific impulse [s]
    float efficiency;         // jet/power efficiency eta (0..1]
    float thrust_max_n;       // hardware thrust limit [N]
    float gimbal_max_rad;     // |pitch|,|yaw| limit [rad]
    float gimbal_rate_rad_s;  // gimbal slew-rate limit [rad/s]
    Vec3  position_b_m;       // thruster location in body frame [m]
};

// Controller commands
struct ThrustCmd {
    float thrust_n;           // requested thrust [N]
    float pitch_rad;          // requested gimbal pitch [rad]
    float yaw_rad;            // requested gimbal yaw [rad]
};

//
struct ThrusterState {
    float pitch_rad;          // where gimbal needs to be
    float yaw_rad;            // where gimbal is
};

struct ThrustOut {
    float thrust_n;           // delivered thrust after all limits [N]
    Vec3  force_b_n;          // force vector, body frame [N]
    Vec3  torque_b_nm;        // torque about centre of mass, body frame [N m]
    float mdot_kg_s;          // propellant mass flow [kg/s]
    float power_used_w;       // electrical power drawn [W]
    bool  power_limited;      // true if available power capped the thrust
};
}   