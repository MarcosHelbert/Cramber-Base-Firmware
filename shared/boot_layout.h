#ifndef BOOT_LAYOUT_H
#define BOOT_LAYOUT_H

#include <stdint.h>

//   0x08000000  sectors 0-1   bootloader (32 KB, flashed once with a programmer)
//   0x08008000  sector 2      application header
//   0x080C0000  sector 10     user preferences  -- kept across updates
//   0x080E0000  sector 11     factory configuration -- kept across updates

#define BOOT_FLASH_START 0x08000000u
#define APP_HEADER_ADDR 0x08008000u
#define APP_HEADER_SIZE 0x200u
#define APP_VECTOR_ADDR (APP_HEADER_ADDR + APP_HEADER_SIZE)
#define APP_REGION_END 0x080C0000u
#define APP_MAX_SIZE (APP_REGION_END - APP_VECTOR_ADDR)

#define APP_HEADER_MAGIC 0x43524D41u // "AMRC"

#define BOOT_REQUEST_MAGIC 0xB007B007u

typedef struct
{
    uint32_t magic;
    uint32_t size;
    uint32_t crc32;
    uint32_t version;
} AppHeader;

#define BOOT_USB_VID 0x1209u
#define BOOT_USB_PID 0x4FFCu

// Bootloader protocol: 64-byte HID reports, no report ID.
//   request:  [0] command  [1] sequence  [2-3] length  [4-7] argument  [8-63] data
//   response: [0] command  [1] sequence  [2] status    [3] 0           [4-7] value
enum BootCommand
{
    BOOT_CMD_INFO = 0x01,   // value = bootloader version, data[0-3] = max image size
    BOOT_CMD_ERASE = 0x02,  // argument = image size; erases what the image needs
    BOOT_CMD_WRITE = 0x03,  // argument = offset in the image, length <= 56 (multiple of 4)
    BOOT_CMD_FINISH = 0x04, // argument = image size, data[0-3] = CRC-32, data[4-7] = version
    BOOT_CMD_BOOT = 0x05,   // restarts into the application
};

enum BootStatus
{
    BOOT_OK = 0,
    BOOT_ERR_COMMAND = 1,
    BOOT_ERR_RANGE = 2,
    BOOT_ERR_FLASH = 3,
    BOOT_ERR_CRC = 4,
    BOOT_ERR_ORDER = 5,
};

#define BOOT_REPORT_SIZE 64u
#define BOOT_DATA_OFFSET 8u
#define BOOT_DATA_MAX (BOOT_REPORT_SIZE - BOOT_DATA_OFFSET)

#endif // BOOT_LAYOUT_H
