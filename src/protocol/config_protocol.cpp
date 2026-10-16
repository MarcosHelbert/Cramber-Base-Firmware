#include "config_protocol.h"

#include <cstring>

#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/iwdg.h>

#include <system/timer.h>

namespace
{
    void packF32(uint8_t out[4], float v)
    {
        std::memcpy(out, &v, sizeof(v));
    }
    void packU32(uint8_t out[4], uint32_t v)
    {
        std::memcpy(out, &v, sizeof(v));
    }
    void packI32(uint8_t out[4], int32_t v)
    {
        std::memcpy(out, &v, sizeof(v));
    }
    void packBool(uint8_t out[4], bool v)
    {
        std::memset(out, 0, 4);
        out[0] = v ? 1 : 0;
    }

    float unpackF32(const uint8_t v[4])
    {
        float f;
        std::memcpy(&f, v, sizeof(f));
        return f;
    }
    uint32_t unpackU32(const uint8_t v[4])
    {
        uint32_t u;
        std::memcpy(&u, v, sizeof(u));
        return u;
    }
    int32_t unpackI32(const uint8_t v[4])
    {
        int32_t i;
        std::memcpy(&i, v, sizeof(i));
        return i;
    }
    bool unpackBool(const uint8_t v[4]) { return v[0] != 0; }
}

ConfigProtocol::ConfigProtocol(Motor &motor, FOC &foc, DRV8301 &drv8301, Axis &axis, ConfigStore &config_store)
    : motor_(motor), foc_(foc), drv8301_(drv8301), axis_(axis), config_store_(config_store) {}

void ConfigProtocol::hidSet(uint8_t report_id, uint8_t const *buffer, uint16_t bufsize)
{
    if (report_id != CONFIG_REPORT_ID_COMMAND)
        return;

    if (bufsize < sizeof(ConfigCommandReport))
        return;

    ConfigCommandReport cmd{};
    std::memcpy(&cmd, buffer, sizeof(cmd));
    lastCommand_ = cmd;

    switch (cmd.command)
    {
    case CMD_GET:
        handleGet_(cmd);
        break;

    case CMD_SET:
        handleSet_(cmd);
        break;

    case CMD_REBOOT:
        respond_(STATUS_OK, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
        scb_reset_system();
        break;

    case CMD_CALIBRATE:
        handleCalibrate_(cmd);
        break;

    case CMD_SAVE:
        handleSave_(cmd);
        break;

    case CMD_FACTORY_RESET:
        handleFactoryReset_(cmd);
        break;

    case CMD_GET_STATUS:
        // TODO: implement once there's a defined status payload beyond
        respond_(STATUS_ERR_BUSY, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
        break;

    default:
        respond_(STATUS_ERR_INVALID_VALUE, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
        break;
    }
}

uint16_t ConfigProtocol::hidGet(uint8_t report_id, uint8_t *buffer)
{
    if (report_id != CONFIG_REPORT_ID_RESPONSE)
        return 0;

    std::memcpy(buffer, &lastResponse_, sizeof(lastResponse_));
    return sizeof(lastResponse_);
}

void ConfigProtocol::respond_(uint8_t status, uint8_t command, uint16_t param_id, uint8_t value_type, const uint8_t value[4], uint8_t sequence)
{
    lastResponse_.status = status;
    lastResponse_.command = command;
    lastResponse_.param_id = param_id;
    lastResponse_.value_type = value_type;

    if (value)
        std::memcpy(lastResponse_.value, value, sizeof(lastResponse_.value));
    else
        std::memset(lastResponse_.value, 0, sizeof(lastResponse_.value));

    lastResponse_.sequence = sequence;
    std::memset(lastResponse_.reserved, 0, sizeof(lastResponse_.reserved));
}

void ConfigProtocol::handleGet_(const ConfigCommandReport &cmd)
{
    uint8_t type = VALUE_NONE;
    uint8_t value[4] = {0};

    if (!readParam_(cmd.param_id, type, value))
    {
        respond_(STATUS_ERR_UNKNOWN_PARAM, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
        return;
    }

    respond_(STATUS_OK, cmd.command, cmd.param_id, type, value, cmd.sequence);
}

void ConfigProtocol::handleCalibrate_(const ConfigCommandReport &cmd)
{
    if (cmd.param_id == PARAM_CALTYPE_MOTOR)
    {
        handleCalibrateMotor_(cmd);
        return;
    }

    if (cmd.param_id == PARAM_CALTYPE_ENCODER)
    {
        handleCalibrateEncoder_(cmd);
        return;
    }

    // TODO: implement CALTYPE_ANTICOGGING (and anything else) isn't implemented yet.
    respond_(STATUS_ERR_BUSY, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
}

void ConfigProtocol::handleCalibrateMotor_(const ConfigCommandReport &cmd)
{
    if (!motor_.arm())
    {
        uint8_t status = STATUS_ERR_CALIBRATION_FAILED;
        switch (motor_.getLastArmFailReason())
        {
        case Motor::ArmFailReason::POWER_OUT_OF_RANGE:
            status = STATUS_ERR_POWER_FAULT;
            break;
        case Motor::ArmFailReason::DRIVER_FAULT:
            status = STATUS_ERR_DRIVER_FAULT;
            break;
        default:
            break;
        }
        respond_(status, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
        return;
    }

    bool ok = motor_.calibrate();
    motor_.disarm();

    if (!ok)
    {
        respond_(STATUS_ERR_CALIBRATION_FAILED, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
        return;
    }

    foc_.updateGainsFromConfig();

    respond_(STATUS_OK, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
}

void ConfigProtocol::handleCalibrateEncoder_(const ConfigCommandReport &cmd)
{
    if (!g_config.motor_calibrated)
    {
        respond_(STATUS_ERR_NOT_CALIBRATED, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
        return;
    }

    if (encoder_calibration_pending_)
    {
        respond_(STATUS_ERR_BUSY, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
        return;
    }

    if (!axis_.startCalibration())
    {
        uint8_t status = STATUS_ERR_CALIBRATION_FAILED;
        switch (motor_.getLastArmFailReason())
        {
        case Motor::ArmFailReason::POWER_OUT_OF_RANGE:
            status = STATUS_ERR_POWER_FAULT;
            break;
        case Motor::ArmFailReason::DRIVER_FAULT:
            status = STATUS_ERR_DRIVER_FAULT;
            break;
        default:
            break;
        }
        respond_(status, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
        return;
    }

    encoder_calibration_pending_ = true;
    respond_(STATUS_IN_PROGRESS, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
}

void ConfigProtocol::poll()
{
    if (!encoder_calibration_pending_)
        return;

    Axis::CalibrationPhase phase = axis_.getCalibrationPhase();
    if (phase != Axis::CalibrationPhase::DONE && phase != Axis::CalibrationPhase::FAILED)
        return;

    axis_.stopCalibration();
    encoder_calibration_pending_ = false;
}

void ConfigProtocol::handleSave_(const ConfigCommandReport &cmd)
{
    iwdg_reset();
    config_store_.save();
    iwdg_reset();

    respond_(STATUS_OK, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
    scb_reset_system();
}

void ConfigProtocol::handleFactoryReset_(const ConfigCommandReport &cmd)
{
    iwdg_reset();
    config_store_.eraseStoredImage();
    iwdg_reset();

    g_config = PersistedConfig{};

    respond_(STATUS_OK, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
}

void ConfigProtocol::handleSet_(const ConfigCommandReport &cmd)
{
    uint8_t status = writeParam_(cmd.param_id, cmd.value_type, cmd.value);
    if (status != STATUS_OK)
    {
        respond_(status, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
        return;
    }

    respond_(STATUS_OK, cmd.command, cmd.param_id, cmd.value_type, cmd.value, cmd.sequence);
}

bool ConfigProtocol::readParam_(uint16_t param_id, uint8_t &out_type, uint8_t out_value[4]) const
{
    const PersistedConfig &c = g_config;

    switch (param_id)
    {
    case PARAM_FW_VERSION:
        out_type = VALUE_U32;
        packU32(out_value, kFirmwareVersion);
        return true;
    case PARAM_CONFIG_SCHEMA_VERSION:
        out_type = VALUE_U32;
        packU32(out_value, kConfigSchemaVersion);
        return true;

    case PARAM_MAX_CURRENT:
        out_type = VALUE_F32;
        packF32(out_value, c.max_current);
        return true;
    case PARAM_VBUS_DIVIDER_RATIO:
        out_type = VALUE_F32;
        packF32(out_value, c.vbus_divider_ratio);
        return true;
    case PARAM_VBUS_MIN_VOLTAGE:
        out_type = VALUE_F32;
        packF32(out_value, c.vbus_min_voltage);
        return true;
    case PARAM_VBUS_MAX_VOLTAGE:
        out_type = VALUE_F32;
        packF32(out_value, c.vbus_max_voltage);
        return true;

    case PARAM_PWM_FREQ_HZ:
        out_type = VALUE_U32;
        packU32(out_value, c.pwm_freq_hz);
        return true;
    case PARAM_PWM_DEADTIME_NS:
        out_type = VALUE_U32;
        packU32(out_value, c.pwm_deadtime_ns);
        return true;

    case PARAM_SHUNT_RESISTANCE:
        out_type = VALUE_F32;
        packF32(out_value, c.shunt_resistance);
        return true;
    case PARAM_CURRENT_AMP_GAIN:
        out_type = VALUE_F32;
        packF32(out_value, c.current_amp_gain);
        return true;

    case PARAM_MOTOR_POLE_PAIRS:
        out_type = VALUE_U32;
        packU32(out_value, c.motor_pole_pairs);
        return true;
    case PARAM_MOTOR_FLUX_LINKAGE:
        out_type = VALUE_F32;
        packF32(out_value, c.motor_flux_linkage);
        return true;
    case PARAM_PHASE_RESISTANCE:
        out_type = VALUE_F32;
        packF32(out_value, c.phase_resistance);
        return true;
    case PARAM_PHASE_INDUCTANCE:
        out_type = VALUE_F32;
        packF32(out_value, c.phase_inductance);
        return true;

    case PARAM_CURRENT_BANDWIDTH_RAD_S:
        out_type = VALUE_F32;
        packF32(out_value, c.current_bandwidth_rad_s);
        return true;
    case PARAM_CURRENT_KP:
        out_type = VALUE_F32;
        packF32(out_value, c.current_kp);
        return true;
    case PARAM_CURRENT_KI:
        out_type = VALUE_F32;
        packF32(out_value, c.current_ki);
        return true;

    case PARAM_CALIBRATION_MAX_VOLTAGE:
        out_type = VALUE_F32;
        packF32(out_value, c.calibration_max_voltage);
        return true;
    case PARAM_CALIBRATION_TEST_CURRENT:
        out_type = VALUE_F32;
        packF32(out_value, c.calibration_test_current);
        return true;
    case PARAM_MOTOR_CALIBRATED:
        out_type = VALUE_BOOL;
        packBool(out_value, c.motor_calibrated);
        return true;

    case PARAM_ENCODER_CPR:
        out_type = VALUE_U32;
        packU32(out_value, c.encoder_cpr);
        return true;
    case PARAM_ENCODER_REVERSE:
        out_type = VALUE_BOOL;
        packBool(out_value, c.encoder_reverse);
        return true;
    case PARAM_ENCODER_MECHANICAL_OFFSET:
        out_type = VALUE_I32;
        packI32(out_value, c.mechanical_offset);
        return true;
    case PARAM_ENCODER_OFFSET_RAD:
        out_type = VALUE_F32;
        packF32(out_value, c.encoder_offset_rad);
        return true;
    case PARAM_ENCODER_CALIBRATED:
        out_type = VALUE_BOOL;
        packBool(out_value, c.encoder_calibrated);
        return true;

    case PARAM_ENCODER_USE_INDEX_Z:
        out_type = VALUE_BOOL;
        packBool(out_value, c.use_index_z);
        return true;
    case PARAM_ENCODER_HOME_ON_BOOT:
        out_type = VALUE_BOOL;
        packBool(out_value, c.home_on_boot);
        return true;

    case PARAM_ANTICOGGING_ENABLED:
        out_type = VALUE_BOOL;
        packBool(out_value, c.anticogging_enabled);
        return true;

    case PARAM_MAX_VELOCITY_RAD_S:
        out_type = VALUE_F32;
        packF32(out_value, c.max_velocity_rad_s);
        return true;
    case PARAM_MAX_ACCELERATION_RAD_S2:
        out_type = VALUE_F32;
        packF32(out_value, c.max_acceleration_rad_s2);
        return true;
    case PARAM_STEERING_RANGE_DEG:
        out_type = VALUE_F32;
        packF32(out_value, c.steering_range_deg);
        return true;

    case PARAM_BRAKE_RESISTOR_ENABLED:
        out_type = VALUE_BOOL;
        packBool(out_value, c.brake_resistor_enabled);
        return true;
    case PARAM_BRAKE_RESISTOR_RESISTANCE_OHM:
        out_type = VALUE_F32;
        packF32(out_value, c.brake_resistor_resistance_ohm);
        return true;
    case PARAM_BRAKE_RESISTOR_POWER_RATING_W:
        out_type = VALUE_F32;
        packF32(out_value, c.brake_resistor_power_rating_w);
        return true;
    case PARAM_BRAKE_RESISTOR_MAX_REGEN_CURRENT:
        out_type = VALUE_F32;
        packF32(out_value, c.brake_resistor_max_regen_current);
        return true;

    case PARAM_DRV_STATUS1:
        out_type = VALUE_U32;
        packU32(out_value, drv8301_.getStatus1());
        return true;
    case PARAM_DRV_STATUS2:
        out_type = VALUE_U32;
        packU32(out_value, drv8301_.getStatus2());
        return true;
    case PARAM_DRV_HAS_FAULT:
        out_type = VALUE_BOOL;
        packBool(out_value, drv8301_.hasFault());
        return true;
    case PARAM_DRV_CONTROL1:
        out_type = VALUE_U32;
        packU32(out_value, drv8301_.getControl1());
        return true;
    case PARAM_DRV_CONTROL2:
        out_type = VALUE_U32;
        packU32(out_value, drv8301_.getControl2());
        return true;
    case PARAM_ARM_FAIL_REASON:
        out_type = VALUE_U32;
        packU32(out_value, static_cast<uint32_t>(motor_.getLastArmFailReason()));
        return true;
    case PARAM_CALIBRATION_FAIL_REASON:
        out_type = VALUE_U32;
        packU32(out_value, static_cast<uint32_t>(motor_.getLastCalibrationFailReason()));
        return true;
    case PARAM_ALIGNMENT_FAIL_REASON:
        out_type = VALUE_U32;
        packU32(out_value, static_cast<uint32_t>(axis_.getLastCalibrationFailReason()));
        return true;
    case PARAM_ENCODER_POSITION_COUNTS:
        out_type = VALUE_I32;
        packI32(out_value, axis_.getEncoderPositionCounts());
        return true;
    case PARAM_ENCODER_CALIBRATION_PHASE:
        out_type = VALUE_U32;
        packU32(out_value, static_cast<uint32_t>(axis_.getCalibrationPhase()));
        return true;

    default:
        return false;
    }
}

uint8_t ConfigProtocol::writeParam_(uint16_t param_id, uint8_t type, const uint8_t value[4])
{
    PersistedConfig &c = g_config;

    switch (param_id)
    {
    case PARAM_MAX_CURRENT:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.max_current = unpackF32(value);
        return STATUS_OK;
    case PARAM_VBUS_DIVIDER_RATIO:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.vbus_divider_ratio = unpackF32(value);
        return STATUS_OK;
    case PARAM_VBUS_MIN_VOLTAGE:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.vbus_min_voltage = unpackF32(value);
        return STATUS_OK;
    case PARAM_VBUS_MAX_VOLTAGE:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.vbus_max_voltage = unpackF32(value);
        return STATUS_OK;

    case PARAM_PWM_FREQ_HZ:
        if (type != VALUE_U32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.pwm_freq_hz = unpackU32(value);
        return STATUS_OK;
    case PARAM_PWM_DEADTIME_NS:
        if (type != VALUE_U32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.pwm_deadtime_ns = unpackU32(value);
        return STATUS_OK;

    case PARAM_SHUNT_RESISTANCE:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.shunt_resistance = unpackF32(value);
        return STATUS_OK;
    case PARAM_CURRENT_AMP_GAIN:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.current_amp_gain = unpackF32(value);
        return STATUS_OK;

    case PARAM_MOTOR_POLE_PAIRS:
        if (type != VALUE_U32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.motor_pole_pairs = unpackU32(value);
        return STATUS_OK;
    case PARAM_MOTOR_FLUX_LINKAGE:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.motor_flux_linkage = unpackF32(value);
        return STATUS_OK;
    case PARAM_PHASE_RESISTANCE:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.phase_resistance = unpackF32(value);
        return STATUS_OK;
    case PARAM_PHASE_INDUCTANCE:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.phase_inductance = unpackF32(value);
        return STATUS_OK;

    case PARAM_CURRENT_BANDWIDTH_RAD_S:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.current_bandwidth_rad_s = unpackF32(value);
        return STATUS_OK;
    case PARAM_CURRENT_KP:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.current_kp = unpackF32(value);
        return STATUS_OK;
    case PARAM_CURRENT_KI:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.current_ki = unpackF32(value);
        return STATUS_OK;

    case PARAM_CALIBRATION_MAX_VOLTAGE:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.calibration_max_voltage = unpackF32(value);
        return STATUS_OK;
    case PARAM_CALIBRATION_TEST_CURRENT:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.calibration_test_current = unpackF32(value);
        return STATUS_OK;
    case PARAM_MOTOR_CALIBRATED:
    {
        if (type != VALUE_BOOL)
            return STATUS_ERR_UNKNOWN_PARAM;
        bool requested = unpackBool(value);
        if (requested && (c.phase_resistance == 0.0f || c.phase_inductance == 0.0f))
            return STATUS_ERR_INVALID_VALUE;
        c.motor_calibrated = requested;
        return STATUS_OK;
    }

    case PARAM_ENCODER_CPR:
        if (type != VALUE_U32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.encoder_cpr = unpackU32(value);
        return STATUS_OK;
    case PARAM_ENCODER_REVERSE:
        if (type != VALUE_BOOL)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.encoder_reverse = unpackBool(value);
        return STATUS_OK;
    case PARAM_ENCODER_MECHANICAL_OFFSET:
        if (type != VALUE_I32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.mechanical_offset = unpackI32(value);
        return STATUS_OK;
    case PARAM_ENCODER_OFFSET_RAD:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.encoder_offset_rad = unpackF32(value);
        return STATUS_OK;
    case PARAM_ENCODER_CALIBRATED:
    {
        if (type != VALUE_BOOL)
            return STATUS_ERR_UNKNOWN_PARAM;
        bool requested = unpackBool(value);
        if (requested && c.encoder_cpr == 0)
            return STATUS_ERR_INVALID_VALUE;
        c.encoder_calibrated = requested;
        return STATUS_OK;
    }

    case PARAM_ENCODER_USE_INDEX_Z:
        if (type != VALUE_BOOL)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.use_index_z = unpackBool(value);
        return STATUS_OK;
    case PARAM_ENCODER_HOME_ON_BOOT:
    {
        if (type != VALUE_BOOL)
            return STATUS_ERR_UNKNOWN_PARAM;
        bool requested = unpackBool(value);
        if (requested && !(c.motor_calibrated && c.encoder_calibrated))
            return STATUS_ERR_NOT_CALIBRATED;
        c.home_on_boot = requested;
        return STATUS_OK;
    }

    case PARAM_ANTICOGGING_ENABLED:
        if (type != VALUE_BOOL)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.anticogging_enabled = unpackBool(value);
        return STATUS_OK;

    case PARAM_MAX_VELOCITY_RAD_S:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.max_velocity_rad_s = unpackF32(value);
        return STATUS_OK;
    case PARAM_MAX_ACCELERATION_RAD_S2:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.max_acceleration_rad_s2 = unpackF32(value);
        return STATUS_OK;
    case PARAM_STEERING_RANGE_DEG:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.steering_range_deg = unpackF32(value);
        return STATUS_OK;

    case PARAM_BRAKE_RESISTOR_ENABLED:
        if (type != VALUE_BOOL)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.brake_resistor_enabled = unpackBool(value);
        return STATUS_OK;
    case PARAM_BRAKE_RESISTOR_RESISTANCE_OHM:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.brake_resistor_resistance_ohm = unpackF32(value);
        return STATUS_OK;
    case PARAM_BRAKE_RESISTOR_POWER_RATING_W:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.brake_resistor_power_rating_w = unpackF32(value);
        return STATUS_OK;
    case PARAM_BRAKE_RESISTOR_MAX_REGEN_CURRENT:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.brake_resistor_max_regen_current = unpackF32(value);
        return STATUS_OK;
    }
}
