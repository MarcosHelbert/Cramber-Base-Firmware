#ifndef VENDOR_HID_REPORT_H
#define VENDOR_HID_REPORT_H

#include <stdint.h>

uint8_t const desc_hid_report_vendor[] = {
    0x06, 0x00, 0xFF, // USAGE_PAGE (Vendor Defined 0xFF00)
    0x09, 0x01, // USAGE (Vendor Usage 1)
    0xA1, 0x01, // COLLECTION (Application)

    // Report ID 21: Config Command (host -> device, 16 bytes)
    0x85, 0x15, //   REPORT_ID (21)
    0x09, 0x02, //   USAGE (Vendor Usage 2)
    0x15, 0x00, //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00, //   LOGICAL_MAXIMUM (255)
    0x75, 0x08, //   REPORT_SIZE (8)
    0x95, 0x10, //   REPORT_COUNT (16)
    0x91, 0x02, //   OUTPUT (Data,Var,Abs)

    // Report ID 23: Config Response (device -> host, 16 bytes)
    0x85, 0x17, //   REPORT_ID (23)
    0x09, 0x03, //   USAGE (Vendor Usage 3)
    0x15, 0x00, //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00, //   LOGICAL_MAXIMUM (255)
    0x75, 0x08, //   REPORT_SIZE (8)
    0x95, 0x10, //   REPORT_COUNT (16)
    0x81, 0x02, //   INPUT (Data,Var,Abs)

    0xC0, // END_COLLECTION
};

#endif // VENDOR_HID_REPORT_H
