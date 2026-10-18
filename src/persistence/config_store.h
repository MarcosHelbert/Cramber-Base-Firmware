#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include <cstdint>

static constexpr uint32_t kConfigSchemaVersion = 1;

struct PersistedConfig
{
    // --- Current and bus limits ---
    float max_current = 2.0f;         // Current limit (A)
    float vbus_divider_ratio = 19.0f; // (R_high + R_low) / R_low
    float vbus_min_voltage = 8.0f;    // Undervoltage threshold (V)
    float vbus_max_voltage = 28.0f;   // Overvoltage threshold (V)

    // --- PWM ---
    uint32_t pwm_freq_hz = 21000;   // Switching frequency (Hz)
    uint32_t pwm_deadtime_ns = 200; // Deadtime between high/low side (ns)

    // --- Current sensing ---
    float shunt_resistance = 0.0005f; // Shunt resistor: 0.5mΩ
    float current_amp_gain = 80.0f;   // 10, 20, 40, 80V/V for DRV8301

    // --- Motor parameters ---
    uint32_t motor_pole_pairs = 15;    // Motor pole pairs
    float motor_flux_linkage = 0.005f; // Rotor flux linkage (Wb), used for back-EMF feedforward
    float phase_resistance = 0.0f;
    float phase_inductance = 0.0f;

    // --- Current loop ---
    float current_bandwidth_rad_s = 1000.0f; // Current loop bandwidth (rad/s)
    float current_kp = 0.0f;                 // Current loop proportional gain
    float current_ki = 0.0f;                 // Current loop integral gain

    // --- Motor calibration ---
    float calibration_max_voltage = 8.0f;  // Voltage used/limited during R/L calibration (V)
    float calibration_test_current = 2.0f; // Target current during phase resistance measurement (A)
    bool motor_calibrated = false;

    // --- Encoder ---
    uint32_t encoder_cpr = 65536;    // Counts per revolution (4 x PPR)
    bool encoder_reverse = false;    // Invert counting direction
    int32_t mechanical_offset = 0;   // Raw encoder count at the index pulse
    float encoder_offset_rad = 0.0f; // Electrical angle offset from alignment
    bool encoder_calibrated = false; // True if the encoder has been calibrated

    // --- Homing ---
    bool use_index_z = true;   // Use the Z/index pulse for absolute homing;
    bool home_on_boot = false; // Run homing automatically at boot

    // --- Force feedback ---
    bool ffb_enabled_on_boot = false;    // Arm the motor and start FFB automatically at boot
    float ffb_slew_rate_a_per_s = 50.0f; // Max rate of change of the FFB current target (A/s).

    // --- Encoder LUT ---
    static constexpr uint32_t kEncoderLutSize = 128;
    float encoder_lut[kEncoderLutSize] = {};
    bool encoder_lut_calibrated = false; // True once the LUT scan has completed successfully

    // --- Anticogging ---
    bool anticogging_enabled = true;              // Apply the anticogging compensation map
    float anticogging_map[kEncoderLutSize] = {};  // Compensating torque (A) to add at each of 128 positions around one mechanical revolution
    bool anticogging_calibrated = false;          // True once the anticogging scan has completed successfully
    float anticogging_scan_velocity_rad_s = 2.0f; // Target angular velocity held during the anticogging calibration scan (rad/s)

    // --- Motion limits ---
    float max_velocity_rad_s = 6.0f;
    float max_acceleration_rad_s2 = 80.0f;
    float steering_range_deg = 900.0f;

    // --- Bumpstop ---
    bool bumpstop_enabled = true;     // Apply the bumpstop torque near +-steering_range_deg/2
    float bumpstop_range_deg = 10.0f; // Angular width of the wall transition zone, each side (deg)
    float bumpstop_stiffness = 1.5f;  // Spring torque scale at full penetration (A)
    float bumpstop_damping = 0.3f;    // Velocity damping scale within the wall (A per rad/s)

    // --- Brake resistor ---
    bool brake_resistor_enabled = true;            // Use the brake resistor at all
    float brake_resistor_resistance_ohm = 2.0f;    // Brake resistor value (ohm)
    float brake_resistor_power_rating_w = 50.0f;   // Brake resistor max power dissipation (W)
    float brake_resistor_max_regen_current = 0.0f; // Regen current allowed back to the supply before braking kicks in (A)
    float brake_resistor_trigger_voltage = 26.0f;  // Only brake above this bus voltage

    // --- Startup sound ---
    bool startup_sound_enabled = true;   // Play the startup melody through the motor at boot
    float startup_sound_volume_a = 0.5f; // Tone current amplitude (A)
};

extern PersistedConfig g_config;

class ConfigStore
{
public:
    ConfigStore() = default;

    void load();
    void save();
    void eraseStoredImage();

private:
    struct StoredImage
    {
        uint32_t magic;
        uint32_t schema_version;
        uint32_t size;
        uint32_t crc32;
        PersistedConfig config;
    };

private:
    uint32_t crc32(const uint8_t *data, uint32_t len);
    uint32_t computeCrc(const StoredImage &image, uint32_t len);
};

#endif // CONFIG_STORE_H