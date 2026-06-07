#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// ENCODER CONFIGURATION
// ============================================================================
#define ENCODER_CPR 4096       // Counts Per Revolution (4 × PPR)
#define ENCODER_REVERSE true   // Invert counting direction
#define ENCODER_UPDATE_HZ 1000 // Update frequency (Hz)

// ============================================================================
// POWER STAGE CONFIGURATION
// ============================================================================
// === Current Sensing (Hardware Specific) ===
#define SHUNT_RESISTANCE 0.0005f // Shunt resistor: 0.5mΩ
#define CURRENT_AMP_GAIN 80.0f   // 10, 20, 40, 80V/V for DRV8301

// === Calibration Parameters ===
#define CALIBRATION_CURRENT 2.0f     // Calibration current (A)
#define CALIBRATION_MAX_VOLTAGE 8.0f // Voltage limit during calibration (V)

#define CALIBRATION_LOCK_DURATION_MS 1000               // Time to wait for rotor to settle (ms)
#define CALIBRATION_SCAN_DISTANCE (16.0f * 3.14159265f) // Distance to scan (rad electrical)
#define CALIBRATION_SCAN_OMEGA (4.0f * 3.14159265f)     // Speed of scan (rad/s electrical)
#define CALIBRATION_RANGE 0.05f                         // Allowable error in CPR/Pole pairs ratio (5%)

// === Control Loop Parameters ===
#define MAX_DI_DT 300.0f     // Maximum di/dt for current control (A/s)
#define MAX_DI2_DT2 25000.0f // Maximum d(di/dt)/dt for jerk-limited torque steps (A/s^2)

// === VBUS Monitoring (Voltage Divider) ===
#define VBUS_DIVIDER_RATIO 19.0f // (R_high + R_low) / R_low
#define VBUS_MIN_VOLTAGE 8.0f    // Undervoltage threshold (V)
#define VBUS_MAX_VOLTAGE 28.0f   // Overvoltage threshold (V)

// ============================================================================
// BRAKING RESISTOR CONFIGURATION
// ============================================================================
#define MAX_REGENERATIVE_CURRENT 0.0f     // Maximum regenerative current before braking resistor engages (A)
#define BRAKE_RESISTOR_RESISTANCE 2.0f    // Resistance of the braking resistor (Ω)
#define BRAKE_RESISTOR_POWER_RATING 50.0f // Power rating of the braking resistor (W)

// ============================================================================
// MOTOR CONFIGURATION
// ============================================================================
// === PWM Settings ===
#define MOTOR_PWM_FREQ_HZ 22000 // Switching frequency (Hz)
#define MOTOR_DEADTIME_NS 200   // Deadtime between high/low side (ns)

// === Control Parameters ===
#define CURRENT_BANDWIDTH_RAD_S 1000.0f // Current loop bandwidth (rad/s)
#define MAX_CURRENT 2.0f                // Current limit (A)

// === Motor Physical Parameters ===
#define MOTOR_POLE_PAIRS 15       // Pole pairs
#define MOTOR_FLUX_LINKAGE 0.005f // Web/Volt-sec (Flux Linkage)

// === Safety Limits ===
#define MAX_VELOCITY_RAD_S 6.0f
#define MAX_ACCELERATION_RAD_S2 80.0f

// IWDG timeout configured in startup (ms)
#define IWDG_TIMEOUT_MS 150

// ============================================================================
// WHEEL CONFIGURATION
// ============================================================================
// === Virtual Bumpstops ===
#define BUMPSTOP_DAMPING 0.5f   // Damping coefficient (A per rad/s approaching wall)
#define BUMPSTOP_RANGE_DEG 5.0f // Transition zone width (degrees before hard limit)

// === Cogging Compensation ===
#define COGGING_COMPENSATION_ENABLED false       // Enable cogging compensation
#define COGGING_COMPENSATION_SAMPLING_POINTS 512 // Number of points in cogging compensation map (mechanical positions)
#define COGGING_COMPENSATION_KP 10.0f
#define COGGING_COMPENSATION_KI 5.0f
#define COGGING_COMPENSATION_KD 0.25f
#define COGGING_COMPENSATION_POS_EPS 0.05f
#define COGGING_COMPENSATION_VEL_EPS 0.15f

#endif // CONFIG_H