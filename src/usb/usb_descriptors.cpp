#include <tusb.h>

#include <ffb_hid_report.h>
#include <vendor_hid_report.h>

#include <ffb/ffb_manager.h>
#include <protocol/config_protocol.h>

extern FFBManager ffb_manager;
extern ConfigProtocol config_protocol;

//--------------------------------------------------------------------+
// Device Identity
//--------------------------------------------------------------------+
#define USB_VID 0x1209 // pid.codes VID (open source hardware)
#define USB_PID 0x4FFB // FFB = Force FeedBack
#define USB_BCD 0x0200 // USB 2.0

//--------------------------------------------------------------------+
// Device Descriptor
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200, // USB 2.0
#ifdef DEBUG
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
#else
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
#endif
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = USB_BCD,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01,
};

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
#ifdef DEBUG
enum
{
    ITF_NUM_JOYSTICK = 0,
    ITF_NUM_CDC_CTRL,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};

#define EPNUM_JOYSTICK 0x81
#define EPNUM_CDC_NOTIF 0x82
#define EPNUM_CDC_IN 0x83
#define EPNUM_CDC_OUT 0x03

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN + TUD_CDC_DESC_LEN)

#else
enum
{
    ITF_NUM_JOYSTICK = 0,
    ITF_NUM_VENDOR,
    ITF_NUM_TOTAL
};

#define EPNUM_JOYSTICK 0x81
#define EPNUM_VENDOR 0x82

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + (2 * TUD_HID_DESC_LEN))
#endif

//--------------------------------------------------------------------+
// HID Descriptor Callbacks
//--------------------------------------------------------------------+
uint8_t const *tud_hid_descriptor_report_cb(uint8_t itf)
{
#ifdef DEBUG
    (void)itf;
    return desc_hid_report_joystick;
#else
    if (itf == ITF_NUM_JOYSTICK)
        return desc_hid_report_joystick;
    if (itf == ITF_NUM_VENDOR)
        return desc_hid_report_vendor;
    return NULL;
#endif
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 500),

#ifdef DEBUG
    TUD_HID_DESCRIPTOR(ITF_NUM_JOYSTICK, 0, HID_ITF_PROTOCOL_NONE,
                       sizeof(desc_hid_report_joystick), EPNUM_JOYSTICK,
                       CFG_TUD_HID_EP_BUFSIZE, 8),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_CTRL, 4, EPNUM_CDC_NOTIF, 8,
                       EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
#else
    TUD_HID_DESCRIPTOR(ITF_NUM_JOYSTICK, 4, HID_ITF_PROTOCOL_NONE,
                       sizeof(desc_hid_report_joystick), EPNUM_JOYSTICK,
                       CFG_TUD_HID_EP_BUFSIZE, 8),
    TUD_HID_DESCRIPTOR(ITF_NUM_VENDOR, 5, HID_ITF_PROTOCOL_NONE,
                       sizeof(desc_hid_report_vendor), EPNUM_VENDOR,
                       CFG_TUD_HID_EP_BUFSIZE, 8),
#endif
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+
char const *string_desc_arr[] = {
    (const char[]){0x09, 0x04}, // 0: Supported language is English (0x0409)
    "Camber",                   // 1: Manufacturer
    "Camber Base Wheel",        // 2: Product
    "CAMBER-DD-2026-001",       // 3: Serials,
#ifdef DEBUG
    "Cramber Debug Interface", // 4: CDC Interface
#else
    "Joystick Interface",      // 4: Joystick Interface
    "Vendor Config Interface", // 5: Vendor Interface
#endif
};

static uint16_t _desc_str[32];
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    uint8_t chr_count;
    if (index == 0)
    {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    }
    else
    {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))
            return NULL;
        const char *str = string_desc_arr[index];
        chr_count = strlen(str);
        if (chr_count > 31)
            chr_count = 31;
        for (uint8_t i = 0; i < chr_count; i++)
            _desc_str[1 + i] = str[i];
    }
    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);
    return _desc_str;
}

//--------------------------------------------------------------------+
// HID Callbacks - Effect Management
//--------------------------------------------------------------------+
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    (void)reqlen;
    (void)report_type;

    if (itf == ITF_NUM_JOYSTICK)
    {
        return ffb_manager.hidGet(report_id, buffer);
    }

#ifndef DEBUG
    if (itf == ITF_NUM_VENDOR)
    {
        return config_protocol.hidGet(report_id, buffer);
    }
#endif

    return 0;
}

void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
    if (report_type != HID_REPORT_TYPE_OUTPUT && report_type != HID_REPORT_TYPE_FEATURE)
        return;

    if (!buffer || bufsize == 0)
        return;

#ifndef DEBUG
    if (itf == ITF_NUM_VENDOR)
    {
        config_protocol.hidSet(report_id, buffer, bufsize);
        return;
    }
#endif

    if (itf == ITF_NUM_JOYSTICK)
    {
        ffb_manager.hidSet(report_id, buffer, bufsize);
    }
}
