#pragma once

namespace prop_arm::models {

struct PropArmDynamicsParams {

    /* ---------- Mass properties ---------- */

    double arm_mass_kg;            // Mass of the arm (excluding motor & prop)
    double motor_mass_kg;          // Mass of motor (without prop)
    double prop_mass_kg;           // Mass of propeller
    double total_mass_kg;          // Total mass (cached or computed)

    /* ---------- Geometry ---------- */

    double arm_length_m;           // Distance from pivot to motor axis
    double prop_radius_m{0.125};   // Propeller radius
    double com_distance_m;         // Distance from pivot to center of mass

    /* ---------- Inertia ---------- */

    double arm_inertia_kg_m2;      // Arm inertia about pivot
    double motor_inertia_kg_m2;    // Motor + prop inertia about rotation axis
    double total_inertia_kg_m2;    // Effective inertia for dynamics

    /* ---------- Gravity ---------- */

    double gravity_m_s2{9.80665};

};

} // namespace prop_arm::models
