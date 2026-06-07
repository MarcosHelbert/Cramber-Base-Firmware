#ifndef VENDOR_HID_REPORT_H
#define VENDOR_HID_REPORT_H

#include <stdint.h>
#include "tusb.h"

uint8_t const desc_hid_report_vendor[] = {
    HID_USAGE_PAGE_N(0xFF00, 2),
    HID_USAGE(0x01),
    HID_COLLECTION(HID_COLLECTION_APPLICATION),

    /* Report ID 2: Detailed Telemetry Input (22 bytes payload) */
    HID_REPORT_ID(2)

    // int16_t x_axis; (2 bytes)
    HID_USAGE(0x30), // X
    HID_LOGICAL_MIN_N(-32768, 2),
    HID_LOGICAL_MAX_N(32767, 2),
    HID_REPORT_SIZE(16),
    HID_REPORT_COUNT(1),
    HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),

    // int32_t raw_encoder; (4 bytes)
    HID_USAGE(0x31), // Y / Encoder
    // Logical Min -2147483648 (0x80000000) -> Tag 0x17, Data 00 00 00 80
    0x17,
    0x00,
    0x00,
    0x00,
    0x80,
    // Logical Max 2147483647 (0x7FFFFFFF) -> Tag 0x27, Data FF FF FF 7F
    0x27,
    0xFF,
    0xFF,
    0xFF,
    0x7F,
    HID_REPORT_SIZE(32),
    HID_REPORT_COUNT(1),
    HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),

    // uint8_t axis_state;
    // uint8_t status_flags; (2 bytes total)
    HID_USAGE(0x32), // Status
    HID_LOGICAL_MIN(0),
    HID_LOGICAL_MAX(255),
    HID_REPORT_SIZE(8),
    HID_REPORT_COUNT(2),
    HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),

    // uint16_t vbus_mv;
    // int16_t i_q_ma;
    // int16_t i_d_ma; (6 bytes total)
    HID_USAGE(0x33), // Electrical
    HID_LOGICAL_MIN_N(-32768, 2),
    HID_LOGICAL_MAX_N(32767, 2),
    HID_REPORT_SIZE(16),
    HID_REPORT_COUNT(3),
    HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),

    // uint8_t motor_temp;
    // uint8_t mosfet_temp; (2 bytes total)
    HID_USAGE(0x34), // Thermal
    HID_LOGICAL_MIN(0),
    HID_LOGICAL_MAX(255),
    HID_REPORT_SIZE(8),
    HID_REPORT_COUNT(2),
    HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),

    // uint16_t torque_setpoint; (2 bytes)
    HID_USAGE(0x35),
    HID_LOGICAL_MIN(0),
    HID_LOGICAL_MAX_N(65535, 2),
    HID_REPORT_SIZE(16),
    HID_REPORT_COUNT(1),
    HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),

    // uint32_t error_code; (4 bytes)
    HID_USAGE(0x36),
    // Logical Min 0
    HID_LOGICAL_MIN(0),
    // Logical Max 4294967295 (0xFFFFFFFF) -> Tag 0x27, Data FF FF FF FF
    0x27,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    HID_REPORT_SIZE(32),
    HID_REPORT_COUNT(1),
    HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),

    /* Report ID 21: Configuration Output (8 bytes payload) */

    HID_REPORT_ID(21)
        HID_USAGE(0x10),
    HID_LOGICAL_MIN(0),
    HID_LOGICAL_MAX(255),
    HID_REPORT_SIZE(8),
    HID_REPORT_COUNT(8), // Generic 8-byte buffer
    HID_OUTPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),

    HID_COLLECTION_END,
};

#endif // VENDOR_HID_REPORT_H