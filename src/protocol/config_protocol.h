#ifndef CONFIG_PROTOCOL_H
#define CONFIG_PROTOCOL_H

#include <cstdint>

#include <axis/axis.h>
#include <driver/drv8301.h>
#include <motor/foc.h>
#include <motor/motor.h>
#include <persistence/config_store.h>

static constexpr uint32_t kFirmwareVersion = 1;

#define CONFIG_REPORT_ID_COMMAND 21
#define CONFIG_REPORT_ID_RESPONSE 23
#define CONFIG_REPORT_SIZE 16

enum ConfigCommand : uint8_t
{
    CMD_GET = 0x01,
    CMD_SET = 0x02,
    CMD_SAVE = 0x03,
    CMD_CALIBRATE = 0x04,
    CMD_GET_STATUS = 0x05,
    CMD_FACTORY_RESET = 0x06,
    CMD_REBOOT = 0x07,
};

enum ConfigStatus : uint8_t
{
    STATUS_OK = 0x00,
    STATUS_ERR_UNKNOWN_PARAM = 0x01,
    STATUS_ERR_BUSY = 0x02,
    STATUS_ERR_INVALID_VALUE = 0x03,
    STATUS_ERR_NOT_CALIBRATED = 0x04,
    STATUS_ERR_CALIBRATION_FAILED = 0x05,
    STATUS_IN_PROGRESS = 0x06,
    STATUS_ERR_POWER_FAULT = 0x07,
    STATUS_ERR_DRIVER_FAULT = 0x08,
};

enum ConfigValueType : uint8_t
{
    VALUE_NONE = 0x00,
    VALUE_U32 = 0x01,
    VALUE_I32 = 0x02,
    VALUE_F32 = 0x03,
    VALUE_BOOL = 0x04,
};

enum ConfigParamId : uint16_t
{
    // 0x0000-0x00FF: meta
    PARAM_FW_VERSION = 0x0001,
    PARAM_CONFIG_SCHEMA_VERSION = 0x0002,

    // 0x0100-0x01FF: current and bus limits
    PARAM_MAX_CURRENT = 0x0100,
    PARAM_VBUS_DIVIDER_RATIO = 0x0101,
    PARAM_VBUS_MIN_VOLTAGE = 0x0102,
    PARAM_VBUS_MAX_VOLTAGE = 0x0103,

    // 0x0200-0x02FF: pwm
    PARAM_PWM_FREQ_HZ = 0x0200,
    PARAM_PWM_DEADTIME_NS = 0x0201,

    // 0x0300-0x03FF: current sensing
    PARAM_SHUNT_RESISTANCE = 0x0300,
    PARAM_CURRENT_AMP_GAIN = 0x0301,

    // 0x0400-0x04FF: motor parameters
    PARAM_MOTOR_POLE_PAIRS = 0x0400,
    PARAM_MOTOR_FLUX_LINKAGE = 0x0401,
    PARAM_PHASE_RESISTANCE = 0x0402, // RW
    PARAM_PHASE_INDUCTANCE = 0x0403, // RW

    // 0x0500-0x05FF: current loop
    PARAM_CURRENT_BANDWIDTH_RAD_S = 0x0500,
    PARAM_CURRENT_KP = 0x0501, // RW
    PARAM_CURRENT_KI = 0x0502, // RW

    // 0x0600-0x06FF: motor calibration
    PARAM_CALIBRATION_MAX_VOLTAGE = 0x0600,
    PARAM_CALIBRATION_TEST_CURRENT = 0x0601,
    PARAM_MOTOR_CALIBRATED = 0x0602, // RW

    // 0x0700-0x07FF: encoder
    PARAM_ENCODER_CPR = 0x0700,
    PARAM_ENCODER_REVERSE = 0x0701,
    PARAM_ENCODER_MECHANICAL_OFFSET = 0x0702,
    PARAM_ENCODER_OFFSET_RAD = 0x0703, // RW
    PARAM_ENCODER_CALIBRATED = 0x0704, // RW

    // 0x0800-0x08FF: homing
    PARAM_ENCODER_USE_INDEX_Z = 0x0800,
    PARAM_ENCODER_HOME_ON_BOOT = 0x0801,

    // 0x0900-0x09FF: anticogging
    PARAM_ANTICOGGING_ENABLED = 0x0900,

    // 0x0A00-0x0AFF: motion limits
    PARAM_MAX_VELOCITY_RAD_S = 0x0A00,
    PARAM_MAX_ACCELERATION_RAD_S2 = 0x0A01,
    PARAM_STEERING_RANGE_DEG = 0x0A02,

    // 0x0B00-0x0BFF: brake resistor
    PARAM_BRAKE_RESISTOR_ENABLED = 0x0B00,
    PARAM_BRAKE_RESISTOR_RESISTANCE_OHM = 0x0B01,
    PARAM_BRAKE_RESISTOR_POWER_RATING_W = 0x0B02,
    PARAM_BRAKE_RESISTOR_MAX_REGEN_CURRENT = 0x0B03,

    // 0xFE00-0xFEFF: calibration type selectors (value of CMD_CALIBRATE)
    PARAM_CALTYPE_MOTOR = 0xFE00,
    PARAM_CALTYPE_ENCODER = 0xFE01,

    // 0xFF00-0xFFFF: diagnostics (read-only)
    PARAM_DRV_STATUS1 = 0xFF00,
    PARAM_DRV_STATUS2 = 0xFF01,
    PARAM_DRV_HAS_FAULT = 0xFF02,
    PARAM_DRV_CONTROL1 = 0xFF03,
    PARAM_DRV_CONTROL2 = 0xFF04,
    PARAM_ARM_FAIL_REASON = 0xFF05,
    PARAM_CALIBRATION_FAIL_REASON = 0xFF06,
    PARAM_ALIGNMENT_FAIL_REASON = 0xFF07,
    PARAM_ENCODER_POSITION_COUNTS = 0xFF08,
    PARAM_ENCODER_CALIBRATION_PHASE = 0xFF09,
};

// Raw 16-byte OUT report layout:
//   byte 0:    command (ConfigCommand)
//   byte 1-2:  param_id (u16 LE)
//   byte 3:    value type (ConfigValueType)
//   byte 4-7:  value (LE bit pattern of u32/i32/f32, or byte 4 only for bool)
//   byte 8:    sequence number (echoed back)
//   byte 9-15: reserved
struct __attribute__((packed)) ConfigCommandReport
{
    uint8_t command;
    uint16_t param_id;
    uint8_t value_type;
    uint8_t value[4];
    uint8_t sequence;
    uint8_t reserved[7];
};

// Raw 16-byte IN report layout:
//   byte 0:     status (ConfigStatus)
//   byte 1:     echoed command
//   byte 2-3:   echoed param_id
//   byte 4:     value type
//   byte 5-8:   value (response for GET, or progress/echo for SET/CALIBRATE)
//   byte 9:     echoed sequence number
//   byte 10-15: reserved
struct __attribute__((packed)) ConfigResponseReport
{
    uint8_t status;
    uint8_t command;
    uint16_t param_id;
    uint8_t value_type;
    uint8_t value[4];
    uint8_t sequence;
    uint8_t reserved[6];
};

static_assert(sizeof(ConfigCommandReport) == CONFIG_REPORT_SIZE, "ConfigCommandReport must be 16 bytes");
static_assert(sizeof(ConfigResponseReport) == CONFIG_REPORT_SIZE, "ConfigResponseReport must be 16 bytes");

class ConfigProtocol
{
public:
    ConfigProtocol(Motor &motor, FOC &foc, DRV8301 &drv8301, Axis &axis, ConfigStore &config_store);
    void hidSet(uint8_t report_id, uint8_t const *buffer, uint16_t bufsize);
    uint16_t hidGet(uint8_t report_id, uint8_t *buffer);

    void poll();

private:
    Motor &motor_;
    FOC &foc_;
    DRV8301 &drv8301_;
    Axis &axis_;
    ConfigStore &config_store_;

    ConfigCommandReport lastCommand_{};
    ConfigResponseReport lastResponse_{};
    bool encoder_calibration_pending_ = false;

    void handleGet_(const ConfigCommandReport &cmd);
    void handleSet_(const ConfigCommandReport &cmd);
    void handleCalibrate_(const ConfigCommandReport &cmd);
    void handleCalibrateMotor_(const ConfigCommandReport &cmd);
    void handleCalibrateEncoder_(const ConfigCommandReport &cmd);
    void handleSave_(const ConfigCommandReport &cmd);
    void handleFactoryReset_(const ConfigCommandReport &cmd);

    bool readParam_(uint16_t param_id, uint8_t &out_type, uint8_t out_value[4]) const;
    uint8_t writeParam_(uint16_t param_id, uint8_t type, const uint8_t value[4]);

    void respond_(uint8_t status, uint8_t command, uint16_t param_id, uint8_t value_type, const uint8_t value[4], uint8_t sequence);
};

#endif // CONFIG_PROTOCOL_H