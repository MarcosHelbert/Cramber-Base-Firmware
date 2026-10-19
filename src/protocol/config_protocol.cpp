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

ConfigProtocol::ConfigProtocol(Motor &motor, FOC &foc, DRV8301 &drv8301, Axis &axis, ConfigStore &config_store, FFBManager &ffb_manager,
                               StartupSound &startup_sound, UserPrefsStore &user_prefs)
    : motor_(motor), foc_(foc), drv8301_(drv8301), axis_(axis), config_store_(config_store), ffb_manager_(ffb_manager),
      startup_sound_(startup_sound), user_prefs_(user_prefs) {}

void ConfigProtocol::init()
{
    captureUserDefaults_();
}

ConfigProtocol::ParamScope ConfigProtocol::scopeOf_(uint16_t param_id)
{
    switch (param_id)
    {
    case PARAM_STEERING_RANGE_DEG:
    case PARAM_BUMPSTOP_ENABLED:
    case PARAM_BUMPSTOP_RANGE_DEG:
    case PARAM_BUMPSTOP_STIFFNESS:
    case PARAM_BUMPSTOP_DAMPING:
    case PARAM_IDLE_SPRING_STIFFNESS:
    case PARAM_IDLE_SPRING_DAMPING:
    case PARAM_FFB_GAIN:
    case PARAM_FFB_DETAIL:
    case PARAM_NATURAL_DAMPING:
    case PARAM_NATURAL_FRICTION:
    case PARAM_NATURAL_INERTIA:
    case PARAM_STARTUP_SOUND_PLAY:
        return ParamScope::USER;

    case PARAM_STARTUP_SOUND_ENABLED:
    case PARAM_IDLE_SPRING_ENABLED:
        return ParamScope::USER_PERSISTENT;

    case PARAM_FW_VERSION:
    case PARAM_CONFIG_SCHEMA_VERSION:
    case PARAM_MOTOR_CALIBRATED:
    case PARAM_ENCODER_CALIBRATED:
    case PARAM_ANTICOGGING_CALIBRATED:
    case PARAM_AXIS_STATE:
    case PARAM_POSITION_REFERENCED:
    case PARAM_HOMING_IN_PROGRESS:
    case PARAM_HOMING_FAIL_REASON:
    case PARAM_DRV_HAS_FAULT:
    case PARAM_ARM_FAIL_REASON:
    case PARAM_GAME_FFB_ACTIVE:
    case PARAM_WHEEL_ANGLE_DEG:
    case PARAM_FFB_ACTIVE:
        return ParamScope::STATUS;

    default:
        return ParamScope::FACTORY;
    }
}

bool ConfigProtocol::readUserDefault_(uint16_t param_id, uint8_t &out_type, uint8_t out_value[4]) const
{
    const UserDefaults &d = user_defaults_;
    out_type = VALUE_F32;
    switch (param_id)
    {
    case PARAM_STEERING_RANGE_DEG:
        packF32(out_value, d.steering_range_deg);
        return true;
    case PARAM_BUMPSTOP_ENABLED:
        out_type = VALUE_BOOL;
        packBool(out_value, d.bumpstop_enabled);
        return true;
    case PARAM_BUMPSTOP_RANGE_DEG:
        packF32(out_value, d.bumpstop_range_deg);
        return true;
    case PARAM_BUMPSTOP_STIFFNESS:
        packF32(out_value, d.bumpstop_hardness);
        return true;
    case PARAM_BUMPSTOP_DAMPING:
        packF32(out_value, d.bumpstop_cushion);
        return true;
    case PARAM_IDLE_SPRING_STIFFNESS:
        packF32(out_value, d.idle_spring_strength);
        return true;
    case PARAM_IDLE_SPRING_DAMPING:
        packF32(out_value, d.idle_spring_smoothing);
        return true;
    case PARAM_FFB_GAIN:
        packF32(out_value, d.ffb_gain);
        return true;
    case PARAM_FFB_DETAIL:
        packF32(out_value, d.ffb_detail);
        return true;
    case PARAM_NATURAL_DAMPING:
        packF32(out_value, d.natural_damping);
        return true;
    case PARAM_NATURAL_FRICTION:
        packF32(out_value, d.natural_friction);
        return true;
    case PARAM_NATURAL_INERTIA:
        packF32(out_value, d.natural_inertia);
        return true;
    default:
        return false;
    }
}

void ConfigProtocol::captureUserDefaults_()
{
    const PersistedConfig &c = g_config;
    user_defaults_.steering_range_deg = c.steering_range_deg;
    user_defaults_.bumpstop_enabled = c.bumpstop_enabled;
    user_defaults_.bumpstop_range_deg = c.bumpstop_range_deg;
    user_defaults_.bumpstop_hardness = c.bumpstop_hardness;
    user_defaults_.bumpstop_cushion = c.bumpstop_cushion;
    user_defaults_.idle_spring_strength = c.idle_spring_strength;
    user_defaults_.idle_spring_smoothing = c.idle_spring_smoothing;
    user_defaults_.ffb_gain = c.ffb_gain;
    user_defaults_.ffb_detail = c.ffb_detail;
    user_defaults_.natural_damping = c.natural_damping;
    user_defaults_.natural_friction = c.natural_friction;
    user_defaults_.natural_inertia = c.natural_inertia;
}

void ConfigProtocol::applyUserDefaults_()
{
    PersistedConfig &c = g_config;
    c.steering_range_deg = user_defaults_.steering_range_deg;
    c.bumpstop_enabled = user_defaults_.bumpstop_enabled;
    c.bumpstop_range_deg = user_defaults_.bumpstop_range_deg;
    c.bumpstop_hardness = user_defaults_.bumpstop_hardness;
    c.bumpstop_cushion = user_defaults_.bumpstop_cushion;
    c.idle_spring_strength = user_defaults_.idle_spring_strength;
    c.idle_spring_smoothing = user_defaults_.idle_spring_smoothing;
    c.ffb_gain = user_defaults_.ffb_gain;
    c.ffb_detail = user_defaults_.ffb_detail;
    c.natural_damping = user_defaults_.natural_damping;
    c.natural_friction = user_defaults_.natural_friction;
    c.natural_inertia = user_defaults_.natural_inertia;
    axis_.initBumpstop();
}

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

    case CMD_GET_DEFAULT:
    {
        uint8_t type = VALUE_NONE;
        uint8_t value[4] = {0};
        if (!readUserDefault_(cmd.param_id, type, value))
        {
            respond_(STATUS_ERR_UNKNOWN_PARAM, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
            break;
        }
        respond_(STATUS_OK, cmd.command, cmd.param_id, type, value, cmd.sequence);
        break;
    }

    case CMD_FACTORY_MODE:
        factory_mode_ = cmd.value[0] != 0;
        if (factory_mode_)
            applyUserDefaults_();
        respond_(STATUS_OK, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
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
        if (!factory_mode_)
        {
            respond_(STATUS_ERR_ACCESS_DENIED, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
            break;
        }
        handleSave_(cmd);
        break;

    case CMD_FACTORY_RESET:
        if (!factory_mode_)
        {
            respond_(STATUS_ERR_ACCESS_DENIED, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
            break;
        }
        handleFactoryReset_(cmd);
        break;

    case CMD_CLEAR_FAULT:
        motor_.clearFault();
        respond_(STATUS_OK, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
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

    if ((!factory_mode_ && scopeOf_(cmd.param_id) == ParamScope::FACTORY) ||
        !readParam_(cmd.param_id, type, value))
    {
        respond_(STATUS_ERR_UNKNOWN_PARAM, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
        return;
    }

    respond_(STATUS_OK, cmd.command, cmd.param_id, type, value, cmd.sequence);
}

void ConfigProtocol::handleCalibrate_(const ConfigCommandReport &cmd)
{
    if (!factory_mode_)
    {
        respond_(STATUS_ERR_ACCESS_DENIED, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
        return;
    }

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

    if (cmd.param_id == PARAM_CALTYPE_HOME)
    {
        handleHome_(cmd);
        return;
    }

    if (cmd.param_id == PARAM_CALTYPE_ANTICOGGING)
    {
        handleCalibrateAnticogging_(cmd);
        return;
    }

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
        if (axis_.getLastCalibrationFailReason() == Axis::CalibrationFailReason::NOT_HOMED)
        {
            status = STATUS_ERR_NOT_CALIBRATED;
        }
        else
        {
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
        }
        respond_(status, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
        return;
    }

    encoder_calibration_pending_ = true;
    respond_(STATUS_IN_PROGRESS, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
}

void ConfigProtocol::handleHome_(const ConfigCommandReport &cmd)
{
    if (homing_pending_)
    {
        respond_(STATUS_ERR_BUSY, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
        return;
    }

    if (!axis_.startHoming())
    {
        uint8_t status = STATUS_ERR_CALIBRATION_FAILED;
        switch (axis_.getLastHomingFailReason())
        {
        case Axis::HomingFailReason::NOT_READY:
            status = STATUS_ERR_NOT_CALIBRATED;
            break;
        case Axis::HomingFailReason::ARM_FAILED:
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
            break;
        default:
            break;
        }
        respond_(status, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
        return;
    }

    homing_pending_ = true;
    respond_(STATUS_IN_PROGRESS, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
}

void ConfigProtocol::handleCalibrateAnticogging_(const ConfigCommandReport &cmd)
{
    if (anticogging_calibration_pending_)
    {
        respond_(STATUS_ERR_BUSY, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
        return;
    }

    if (!axis_.startAnticoggingCalibration())
    {
        uint8_t status = STATUS_ERR_CALIBRATION_FAILED;
        switch (axis_.getLastAnticoggingFailReason())
        {
        case Axis::AnticoggingFailReason::NOT_READY:
        case Axis::AnticoggingFailReason::NOT_HOMED:
            status = STATUS_ERR_NOT_CALIBRATED;
            break;
        case Axis::AnticoggingFailReason::ARM_FAILED:
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
            break;
        default:
            break;
        }
        respond_(status, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
        return;
    }

    anticogging_calibration_pending_ = true;
    respond_(STATUS_IN_PROGRESS, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
}

void ConfigProtocol::poll()
{
    if (sound_pending_ && !startup_sound_.isPlaying())
    {
        motor_.disarm();
        sound_pending_ = false;
    }

    if (encoder_calibration_pending_)
    {
        Axis::CalibrationPhase phase = axis_.getCalibrationPhase();
        if (phase == Axis::CalibrationPhase::DONE || phase == Axis::CalibrationPhase::FAILED)
        {
            axis_.stopCalibration();
            encoder_calibration_pending_ = false;
        }
    }

    if (homing_pending_)
    {
        if (axis_.isHomingDone() || axis_.getLastHomingFailReason() != Axis::HomingFailReason::NONE)
        {
            axis_.stopHoming();
            homing_pending_ = false;
        }
    }

    if (anticogging_calibration_pending_)
    {
        Axis::AnticoggingPhase phase = axis_.getAnticoggingPhase();
        if (phase == Axis::AnticoggingPhase::DONE || phase == Axis::AnticoggingPhase::FAILED)
        {
            axis_.stopAnticoggingCalibration();
            anticogging_calibration_pending_ = false;
        }
    }
}

void ConfigProtocol::prepareFlashWrite_()
{
    ffb_manager_.stopFFB();
    if (axis_.getState() == Axis::State::RUNNING)
        axis_.setRunning(false);
    motor_.disarm();

    iwdg_set_period_ms(5000);
    iwdg_reset();
}

void ConfigProtocol::handleSave_(const ConfigCommandReport &cmd)
{
    if (axis_.getState() != Axis::State::IDLE && axis_.getState() != Axis::State::RUNNING)
    {
        respond_(STATUS_ERR_BUSY, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
        return;
    }

    prepareFlashWrite_();
    config_store_.save();

    respond_(STATUS_OK, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
    scb_reset_system();
}

void ConfigProtocol::handleFactoryReset_(const ConfigCommandReport &cmd)
{
    if (axis_.getState() != Axis::State::IDLE && axis_.getState() != Axis::State::RUNNING)
    {
        respond_(STATUS_ERR_BUSY, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
        return;
    }

    prepareFlashWrite_();
    config_store_.eraseStoredImage();
    user_prefs_.erase();

    respond_(STATUS_OK, cmd.command, cmd.param_id, VALUE_NONE, nullptr, cmd.sequence);
    scb_reset_system();
}

void ConfigProtocol::handleSet_(const ConfigCommandReport &cmd)
{
    const ParamScope scope = scopeOf_(cmd.param_id);
    uint8_t status = STATUS_OK;

    if (!factory_mode_ && scope == ParamScope::FACTORY)
        status = STATUS_ERR_UNKNOWN_PARAM;
    else if (!factory_mode_ && scope == ParamScope::STATUS)
        status = STATUS_ERR_ACCESS_DENIED;
    else
        status = writeParam_(cmd.param_id, cmd.value_type, cmd.value);

    if (status == STATUS_OK && scope == ParamScope::USER && factory_mode_)
        captureUserDefaults_();

    if (status == STATUS_OK && scope == ParamScope::USER_PERSISTENT && !factory_mode_)
    {
        if (!user_prefs_.save(g_config, motor_.getState() == Motor::MotorState::ARMED))
            status = STATUS_ERR_BUSY;
    }

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
    case PARAM_ANTICOGGING_CALIBRATED:
        out_type = VALUE_BOOL;
        packBool(out_value, c.anticogging_calibrated);
        return true;
    case PARAM_ANTICOGGING_SCAN_VELOCITY_RAD_S:
        out_type = VALUE_F32;
        packF32(out_value, c.anticogging_scan_velocity_rad_s);
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
    case PARAM_BRAKE_RESISTOR_TRIGGER_VOLTAGE:
        out_type = VALUE_F32;
        packF32(out_value, c.brake_resistor_trigger_voltage);
        return true;

    case PARAM_FFB_ENABLED_ON_BOOT:
        out_type = VALUE_BOOL;
        packBool(out_value, c.ffb_enabled_on_boot);
        return true;
    case PARAM_FFB_ACTIVE:
        out_type = VALUE_BOOL;
        packBool(out_value, motor_.getState() == Motor::MotorState::ARMED && ffb_manager_.isFFBActive());
        return true;
    case PARAM_FFB_SLEW_RATE_A_PER_S:
        out_type = VALUE_F32;
        packF32(out_value, c.ffb_slew_rate_a_per_s);
        return true;
    case PARAM_IDLE_SPRING_ENABLED:
        out_type = VALUE_BOOL;
        packBool(out_value, c.idle_spring_enabled);
        return true;
    case PARAM_IDLE_SPRING_STIFFNESS:
        out_type = VALUE_F32;
        packF32(out_value, c.idle_spring_strength);
        return true;
    case PARAM_IDLE_SPRING_DAMPING:
        out_type = VALUE_F32;
        packF32(out_value, c.idle_spring_smoothing);
        return true;
    case PARAM_FFB_GAIN:
        out_type = VALUE_F32;
        packF32(out_value, c.ffb_gain);
        return true;
    case PARAM_FFB_DETAIL:
        out_type = VALUE_F32;
        packF32(out_value, c.ffb_detail);
        return true;
    case PARAM_NATURAL_DAMPING:
        out_type = VALUE_F32;
        packF32(out_value, c.natural_damping);
        return true;
    case PARAM_NATURAL_FRICTION:
        out_type = VALUE_F32;
        packF32(out_value, c.natural_friction);
        return true;
    case PARAM_NATURAL_INERTIA:
        out_type = VALUE_F32;
        packF32(out_value, c.natural_inertia);
        return true;

    case PARAM_BUMPSTOP_ENABLED:
        out_type = VALUE_BOOL;
        packBool(out_value, c.bumpstop_enabled);
        return true;
    case PARAM_BUMPSTOP_RANGE_DEG:
        out_type = VALUE_F32;
        packF32(out_value, c.bumpstop_range_deg);
        return true;
    case PARAM_BUMPSTOP_STIFFNESS:
        out_type = VALUE_F32;
        packF32(out_value, c.bumpstop_hardness);
        return true;
    case PARAM_BUMPSTOP_DAMPING:
        out_type = VALUE_F32;
        packF32(out_value, c.bumpstop_cushion);
        return true;

    case PARAM_STARTUP_SOUND_ENABLED:
        out_type = VALUE_BOOL;
        packBool(out_value, c.startup_sound_enabled);
        return true;
    case PARAM_STARTUP_SOUND_VOLUME_A:
        out_type = VALUE_F32;
        packF32(out_value, c.startup_sound_volume_a);
        return true;
    case PARAM_STARTUP_SOUND_PLAY:
        out_type = VALUE_BOOL;
        packBool(out_value, startup_sound_.isPlaying());
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
    case PARAM_HOMING_FAIL_REASON:
        out_type = VALUE_U32;
        packU32(out_value, static_cast<uint32_t>(axis_.getLastHomingFailReason()));
        return true;
    case PARAM_HOMING_IN_PROGRESS:
        out_type = VALUE_BOOL;
        packBool(out_value, axis_.getState() == Axis::State::HOMING);
        return true;
    case PARAM_AXIS_STATE:
        out_type = VALUE_U32;
        packU32(out_value, static_cast<uint32_t>(axis_.getState()));
        return true;
    case PARAM_POSITION_REFERENCED:
        out_type = VALUE_BOOL;
        packBool(out_value, axis_.isPositionReferenced());
        return true;
    case PARAM_CALIBRATION_SCAN_DELTA_COUNTS:
        out_type = VALUE_I32;
        packI32(out_value, axis_.getLastScanDeltaCounts());
        return true;
    case PARAM_SOUND_PEAK_VOLTAGE:
        out_type = VALUE_F32;
        packF32(out_value, startup_sound_.getPeakVoltage());
        return true;
    case PARAM_SOUND_PEAK_CURRENT:
        out_type = VALUE_F32;
        packF32(out_value, startup_sound_.getPeakCurrent());
        return true;
    case PARAM_GAME_FFB_ACTIVE:
        out_type = VALUE_BOOL;
        packBool(out_value, ffb_manager_.isGameDriving(millis()));
        return true;
    case PARAM_WHEEL_ANGLE_DEG:
        out_type = VALUE_F32;
        packF32(out_value, axis_.getWheelAngleRad() * (180.0f / 3.14159265f));
        return true;
    case PARAM_ANTICOGGING_FAIL_REASON:
        out_type = VALUE_U32;
        packU32(out_value, static_cast<uint32_t>(axis_.getLastAnticoggingFailReason()));
        return true;
    case PARAM_ANTICOGGING_PHASE:
        out_type = VALUE_U32;
        packU32(out_value, static_cast<uint32_t>(axis_.getAnticoggingPhase()));
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

        if (c.phase_resistance > 0.0f && c.phase_inductance > 0.0f)
        {
            c.current_kp = c.phase_inductance * c.current_bandwidth_rad_s;
            c.current_ki = c.phase_resistance * c.current_bandwidth_rad_s;
            foc_.updateGainsFromConfig();
        }
        return STATUS_OK;
    case PARAM_CURRENT_KP:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.current_kp = unpackF32(value);
        foc_.updateGainsFromConfig();
        return STATUS_OK;
    case PARAM_CURRENT_KI:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.current_ki = unpackF32(value);
        foc_.updateGainsFromConfig();
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
    case PARAM_ANTICOGGING_SCAN_VELOCITY_RAD_S:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.anticogging_scan_velocity_rad_s = unpackF32(value);
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
    {
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        const float range = unpackF32(value);
        if (!(range >= 90.0f && range <= 2700.0f))
            return STATUS_ERR_INVALID_VALUE;
        c.steering_range_deg = range;
        axis_.initBumpstop();
        return STATUS_OK;
    }

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
    case PARAM_BRAKE_RESISTOR_TRIGGER_VOLTAGE:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.brake_resistor_trigger_voltage = unpackF32(value);
        return STATUS_OK;

    case PARAM_FFB_ENABLED_ON_BOOT:
    {
        if (type != VALUE_BOOL)
            return STATUS_ERR_UNKNOWN_PARAM;
        bool requested = unpackBool(value);
        if (requested && !(c.motor_calibrated && c.encoder_calibrated))
            return STATUS_ERR_NOT_CALIBRATED;
        c.ffb_enabled_on_boot = requested;
        return STATUS_OK;
    }
    case PARAM_FFB_ACTIVE:
    {
        if (type != VALUE_BOOL)
            return STATUS_ERR_UNKNOWN_PARAM;
        bool requested = unpackBool(value);

        if (!requested)
        {
            ffb_manager_.stopFFB();
            motor_.disarm();
            axis_.setRunning(false);
            return STATUS_OK;
        }

        if (!(c.motor_calibrated && c.encoder_calibrated))
            return STATUS_ERR_NOT_CALIBRATED;

        if (!axis_.isPositionReferenced())
            return STATUS_ERR_NOT_CALIBRATED;

        if (!motor_.arm())
        {
            switch (motor_.getLastArmFailReason())
            {
            case Motor::ArmFailReason::POWER_OUT_OF_RANGE:
                return STATUS_ERR_POWER_FAULT;
            case Motor::ArmFailReason::DRIVER_FAULT:
                return STATUS_ERR_DRIVER_FAULT;
            default:
                return STATUS_ERR_CALIBRATION_FAILED;
            }
        }

        axis_.setRunning(true);
        ffb_manager_.startFFB();
        return STATUS_OK;
    }
    case PARAM_FFB_SLEW_RATE_A_PER_S:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.ffb_slew_rate_a_per_s = unpackF32(value);
        return STATUS_OK;
    case PARAM_IDLE_SPRING_ENABLED:
        if (type != VALUE_BOOL)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.idle_spring_enabled = unpackBool(value);
        return STATUS_OK;
    case PARAM_IDLE_SPRING_STIFFNESS:
    {
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        const float v = unpackF32(value);
        if (!(v >= 0.0f && v <= 1.0f))
            return STATUS_ERR_INVALID_VALUE;
        c.idle_spring_strength = v;
        return STATUS_OK;
    }
    case PARAM_IDLE_SPRING_DAMPING:
    {
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        const float v = unpackF32(value);
        if (!(v >= 0.0f && v <= 1.0f))
            return STATUS_ERR_INVALID_VALUE;
        c.idle_spring_smoothing = v;
        return STATUS_OK;
    }
    case PARAM_FFB_GAIN:
    {
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        const float v = unpackF32(value);
        if (v < 0.0f || v > 1.0f)
            return STATUS_ERR_INVALID_VALUE;
        c.ffb_gain = v;
        return STATUS_OK;
    }
    case PARAM_FFB_DETAIL:
    {
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        const float v = unpackF32(value);
        if (v < 0.0f || v > 1.0f)
            return STATUS_ERR_INVALID_VALUE;
        c.ffb_detail = v;
        return STATUS_OK;
    }
    case PARAM_NATURAL_DAMPING:
    {
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        const float v = unpackF32(value);
        if (v < 0.0f || v > 1.0f)
            return STATUS_ERR_INVALID_VALUE;
        c.natural_damping = v;
        return STATUS_OK;
    }
    case PARAM_NATURAL_FRICTION:
    {
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        const float v = unpackF32(value);
        if (v < 0.0f || v > 1.0f)
            return STATUS_ERR_INVALID_VALUE;
        c.natural_friction = v;
        return STATUS_OK;
    }
    case PARAM_NATURAL_INERTIA:
    {
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        const float v = unpackF32(value);
        if (v < 0.0f || v > 1.0f)
            return STATUS_ERR_INVALID_VALUE;
        c.natural_inertia = v;
        return STATUS_OK;
    }

    case PARAM_BUMPSTOP_ENABLED:
        if (type != VALUE_BOOL)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.bumpstop_enabled = unpackBool(value);
        axis_.initBumpstop();
        return STATUS_OK;
    case PARAM_BUMPSTOP_RANGE_DEG:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.bumpstop_range_deg = unpackF32(value);
        axis_.initBumpstop();
        return STATUS_OK;
    case PARAM_BUMPSTOP_STIFFNESS:
    {
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        const float v = unpackF32(value);
        if (!(v >= 0.0f && v <= 1.0f))
            return STATUS_ERR_INVALID_VALUE;
        c.bumpstop_hardness = v;
        axis_.initBumpstop();
        return STATUS_OK;
    }
    case PARAM_BUMPSTOP_DAMPING:
    {
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        const float v = unpackF32(value);
        if (!(v >= 0.0f && v <= 1.0f))
            return STATUS_ERR_INVALID_VALUE;
        c.bumpstop_cushion = v;
        axis_.initBumpstop();
        return STATUS_OK;
    }

    case PARAM_STARTUP_SOUND_ENABLED:
        if (type != VALUE_BOOL)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.startup_sound_enabled = unpackBool(value);
        return STATUS_OK;
    case PARAM_STARTUP_SOUND_VOLUME_A:
        if (type != VALUE_F32)
            return STATUS_ERR_UNKNOWN_PARAM;
        c.startup_sound_volume_a = unpackF32(value);
        return STATUS_OK;
    case PARAM_STARTUP_SOUND_PLAY:
    {
        if (type != VALUE_BOOL)
            return STATUS_ERR_UNKNOWN_PARAM;
        if (!unpackBool(value))
            return STATUS_OK;
        if (!c.motor_calibrated)
            return STATUS_ERR_NOT_CALIBRATED;
        if (sound_pending_ || startup_sound_.isPlaying())
            return STATUS_ERR_BUSY;

        if (axis_.getState() == Axis::State::RUNNING && motor_.getState() == Motor::MotorState::ARMED)
            return startup_sound_.start() ? STATUS_OK : STATUS_ERR_NOT_CALIBRATED;

        if (axis_.getState() != Axis::State::IDLE || motor_.getState() == Motor::MotorState::ARMED)
            return STATUS_ERR_BUSY;
        if (!motor_.arm())
        {
            switch (motor_.getLastArmFailReason())
            {
            case Motor::ArmFailReason::POWER_OUT_OF_RANGE:
                return STATUS_ERR_POWER_FAULT;
            case Motor::ArmFailReason::DRIVER_FAULT:
                return STATUS_ERR_DRIVER_FAULT;
            default:
                return STATUS_ERR_CALIBRATION_FAILED;
            }
        }
        if (!startup_sound_.start())
        {
            motor_.disarm();
            return STATUS_ERR_NOT_CALIBRATED;
        }
        sound_pending_ = true;
        return STATUS_OK;
    }

    default:
        return STATUS_ERR_UNKNOWN_PARAM;
    }
}
