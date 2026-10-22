#include <cstring>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/usb/dwc/otg_fs.h>

#include <tusb.h>

#include <boot_layout.h>

namespace
{
    constexpr uint32_t kBootloaderVersion = 1;
    constexpr uint32_t kRamStart = 0x20000000u;
    constexpr uint32_t kRamEnd = 0x20020000u;

    struct Sector
    {
        uint8_t number;
        uint32_t start;
    };
    constexpr Sector kAppSectors[] = {
        {2, 0x08008000u},
        {3, 0x0800C000u},
        {4, 0x08010000u},
        {5, 0x08020000u},
        {6, 0x08040000u},
        {7, 0x08060000u},
        {8, 0x08080000u},
        {9, 0x080A0000u},
    };

    volatile uint32_t g_ms = 0;

    // ── image checks ────────────────────────────────────────────────────

    uint32_t crc32(const uint8_t *data, uint32_t length)
    {
        uint32_t crc = 0xFFFFFFFFu;
        for (uint32_t i = 0; i < length; i++)
        {
            crc ^= data[i];
            for (int bit = 0; bit < 8; bit++)
                crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
        return ~crc;
    }

    bool applicationValid()
    {
        const auto *header = reinterpret_cast<const AppHeader *>(APP_HEADER_ADDR);
        if (header->magic != APP_HEADER_MAGIC)
            return false;

        const auto *vectors = reinterpret_cast<const uint32_t *>(APP_VECTOR_ADDR);
        const uint32_t stack = vectors[0];
        const uint32_t reset = vectors[1];
        if (stack < kRamStart || stack > kRamEnd || reset < APP_VECTOR_ADDR || reset >= APP_REGION_END)
            return false;

        if (header->size == 0)
            return true;
        if (header->size > APP_MAX_SIZE)
            return false;
        return crc32(reinterpret_cast<const uint8_t *>(APP_VECTOR_ADDR), header->size) == header->crc32;
    }

    bool takeUpdateRequest()
    {
        rcc_periph_clock_enable(RCC_PWR);
        const bool requested = RTC_BKPXR(0) == BOOT_REQUEST_MAGIC;
        if (requested)
        {
            pwr_disable_backup_domain_write_protect();
            RTC_BKPXR(0) = 0;
            pwr_enable_backup_domain_write_protect();
        }
        return requested;
    }

    [[noreturn]] void startApplication()
    {
        const auto *vectors = reinterpret_cast<const uint32_t *>(APP_VECTOR_ADDR);
        SCB_VTOR = APP_VECTOR_ADDR;
        __asm__ volatile("msr msp, %0\n"
                         "bx %1\n"
                         :
                         : "r"(vectors[0]), "r"(vectors[1]));
        __builtin_unreachable();
    }

    void relaxWatchdog()
    {
        if (RCC_CSR & RCC_CSR_LSIRDY)
        {
            iwdg_set_period_ms(20000);
            iwdg_reset();
        }
    }

    // ── flash ───────────────────────────────────────────────────────────

    uint32_t g_erased_size = 0;

    bool flashOk()
    {
        constexpr uint32_t kErrors = FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR;
        const bool ok = (FLASH_SR & kErrors) == 0;
        FLASH_SR = kErrors;
        return ok;
    }

    uint8_t eraseFor(uint32_t image_size)
    {
        if (image_size == 0 || image_size > APP_MAX_SIZE)
            return BOOT_ERR_RANGE;

        g_erased_size = 0;
        const uint32_t end = APP_VECTOR_ADDR + image_size;
        flash_unlock();
        for (const Sector &sector : kAppSectors)
        {
            if (sector.start >= end)
                break;
            iwdg_reset();
            flash_erase_sector(sector.number, FLASH_CR_PROGRAM_X32);
            if (!flashOk())
            {
                flash_lock();
                return BOOT_ERR_FLASH;
            }
        }
        flash_lock();
        g_erased_size = image_size;
        return BOOT_OK;
    }

    uint8_t write(uint32_t offset, const uint8_t *data, uint32_t length)
    {
        if (g_erased_size == 0)
            return BOOT_ERR_ORDER;
        if (length == 0 || length > BOOT_DATA_MAX || (length % 4) != 0 || (offset % 4) != 0 ||
            offset + length > ((g_erased_size + 3u) & ~3u))
            return BOOT_ERR_RANGE;

        const uint32_t address = APP_VECTOR_ADDR + offset;
        flash_unlock();
        for (uint32_t i = 0; i < length; i += 4)
        {
            uint32_t word;
            std::memcpy(&word, data + i, 4);
            flash_program_word(address + i, word);
        }
        flash_lock();

        if (!flashOk() || std::memcmp(reinterpret_cast<const void *>(address), data, length) != 0)
            return BOOT_ERR_FLASH;
        return BOOT_OK;
    }

    uint8_t finish(uint32_t image_size, uint32_t crc, uint32_t version)
    {
        if (g_erased_size == 0)
            return BOOT_ERR_ORDER;
        if (image_size != g_erased_size)
            return BOOT_ERR_RANGE;
        if (crc32(reinterpret_cast<const uint8_t *>(APP_VECTOR_ADDR), image_size) != crc)
            return BOOT_ERR_CRC;

        flash_unlock();
        flash_program_word(APP_HEADER_ADDR + offsetof(AppHeader, size), image_size);
        flash_program_word(APP_HEADER_ADDR + offsetof(AppHeader, crc32), crc);
        flash_program_word(APP_HEADER_ADDR + offsetof(AppHeader, version), version);
        flash_program_word(APP_HEADER_ADDR + offsetof(AppHeader, magic), APP_HEADER_MAGIC);
        flash_lock();
        if (!flashOk() || !applicationValid())
            return BOOT_ERR_FLASH;
        g_erased_size = 0;
        return BOOT_OK;
    }

    // ── protocol ────────────────────────────────────────────────────────

    uint8_t g_request[BOOT_REPORT_SIZE];
    volatile bool g_request_pending = false;
    uint8_t g_response[BOOT_REPORT_SIZE];
    bool g_response_pending = false;
    bool g_restart_pending = false;
    uint32_t g_restart_at_ms = 0;

    uint32_t readU32(const uint8_t *p)
    {
        uint32_t v;
        std::memcpy(&v, p, 4);
        return v;
    }

    void handleRequest()
    {
        const uint8_t command = g_request[0];
        const uint16_t length = static_cast<uint16_t>(g_request[2] | (g_request[3] << 8));
        const uint32_t argument = readU32(&g_request[4]);
        const uint8_t *data = &g_request[BOOT_DATA_OFFSET];

        uint8_t status = BOOT_OK;
        uint32_t value = 0;
        std::memset(g_response, 0, sizeof(g_response));

        switch (command)
        {
        case BOOT_CMD_INFO:
        {
            value = kBootloaderVersion;
            const uint32_t max_size = APP_MAX_SIZE;
            std::memcpy(&g_response[BOOT_DATA_OFFSET], &max_size, 4);
            break;
        }
        case BOOT_CMD_ERASE:
            status = eraseFor(argument);
            break;
        case BOOT_CMD_WRITE:
            status = write(argument, data, length);
            break;
        case BOOT_CMD_FINISH:
            status = finish(argument, readU32(data), readU32(data + 4));
            break;
        case BOOT_CMD_BOOT:
            g_restart_pending = true;
            g_restart_at_ms = g_ms + 50;
            break;
        default:
            status = BOOT_ERR_COMMAND;
            break;
        }

        g_response[0] = command;
        g_response[1] = g_request[1];
        g_response[2] = status;
        std::memcpy(&g_response[4], &value, 4);
        g_response_pending = true;
    }

    // ── USB ─────────────────────────────────────────────────────────────

    void usbInit()
    {
        rcc_periph_reset_pulse(RST_OTGFS);
        rcc_periph_clock_enable(RCC_GPIOA);
        rcc_periph_clock_enable(RCC_OTGFS);
        gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
        gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

        OTG_FS_GCCFG |= OTG_GCCFG_PWRDWN;
        OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS;
        OTG_FS_GCCFG &= ~(OTG_GCCFG_VBUSASEN | OTG_GCCFG_VBUSBSEN);
        OTG_FS_DCTL |= OTG_DCTL_SDIS;

        nvic_enable_irq(NVIC_OTG_FS_IRQ);
        tusb_init();
        OTG_FS_DCTL &= ~OTG_DCTL_SDIS;
    }

    void systickInit()
    {
        systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
        systick_set_reload(168000 - 1);
        systick_interrupt_enable();
        systick_counter_enable();
    }
}

// ── entry ───────────────────────────────────────────────────────────────

int main()
{
    relaxWatchdog();

    const bool update_requested = takeUpdateRequest();
    if (!update_requested && applicationValid())
        startApplication();

    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    systickInit();
    usbInit();

    while (true)
    {
        iwdg_reset();
        tud_task();

        if (g_request_pending && !g_response_pending)
        {
            handleRequest();
            g_request_pending = false;
        }
        if (g_response_pending && tud_hid_ready() && tud_hid_report(0, g_response, sizeof(g_response)))
            g_response_pending = false;
        if (g_restart_pending && !g_response_pending && static_cast<int32_t>(g_ms - g_restart_at_ms) >= 0)
            scb_reset_system();
    }
}

extern "C"
{
    void sys_tick_handler(void) { g_ms++; }
    void otg_fs_isr(void) { tud_int_handler(0); }
    uint32_t tusb_time_millis_api(void) { return g_ms; }

    // ── TinyUSB descriptors and callbacks ───────────────────────────────

    static const uint8_t kReportDescriptor[] = {
        TUD_HID_REPORT_DESC_GENERIC_INOUT(BOOT_REPORT_SIZE)};

    static const tusb_desc_device_t kDeviceDescriptor = {
        .bLength = sizeof(tusb_desc_device_t),
        .bDescriptorType = TUSB_DESC_DEVICE,
        .bcdUSB = 0x0200,
        .bDeviceClass = 0x00,
        .bDeviceSubClass = 0x00,
        .bDeviceProtocol = 0x00,
        .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
        .idVendor = BOOT_USB_VID,
        .idProduct = BOOT_USB_PID,
        .bcdDevice = 0x0100,
        .iManufacturer = 0x01,
        .iProduct = 0x02,
        .iSerialNumber = 0x03,
        .bNumConfigurations = 0x01,
    };

    enum
    {
        ITF_NUM_HID = 0,
        ITF_NUM_TOTAL
    };
    static const uint8_t kConfigDescriptor[] = {
        TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN, 0, 100),
        TUD_HID_INOUT_DESCRIPTOR(ITF_NUM_HID, 0, HID_ITF_PROTOCOL_NONE, sizeof(kReportDescriptor), 0x01, 0x81,
                                 BOOT_REPORT_SIZE, 1),
    };

    uint8_t const *tud_descriptor_device_cb(void) { return reinterpret_cast<uint8_t const *>(&kDeviceDescriptor); }
    uint8_t const *tud_descriptor_configuration_cb(uint8_t) { return kConfigDescriptor; }
    uint8_t const *tud_hid_descriptor_report_cb(uint8_t) { return kReportDescriptor; }

    static const char *serialNumber()
    {
        static char serial[] = "CRAMBER-DD-000000000000";
        const uint32_t words[3] = {DESIG_UNIQUE_ID0, DESIG_UNIQUE_ID1, DESIG_UNIQUE_ID2};
        uint64_t hash = 0xcbf29ce484222325ull;
        for (uint32_t word : words)
        {
            for (int i = 0; i < 4; i++)
            {
                hash ^= (word >> (8 * i)) & 0xFFu;
                hash *= 0x100000001b3ull;
            }
        }
        static const char kHex[] = "0123456789ABCDEF";
        for (int i = 0; i < 12; i++)
            serial[11 + i] = kHex[(hash >> (4 * (11 - i))) & 0xFu];
        return serial;
    }

    static uint16_t g_string[32];
    uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t)
    {
        static const char *const kStrings[] = {nullptr, "Cramber", "Cramber DD Bootloader"};
        uint8_t count;
        if (index == 0)
        {
            g_string[1] = 0x0409;
            count = 1;
        }
        else
        {
            const char *text = index == 3 ? serialNumber() : (index < 3 ? kStrings[index] : nullptr);
            if (text == nullptr)
                return nullptr;
            count = static_cast<uint8_t>(std::strlen(text));
            if (count > 31)
                count = 31;
            for (uint8_t i = 0; i < count; i++)
                g_string[1 + i] = static_cast<uint16_t>(text[i]);
        }
        g_string[0] = static_cast<uint16_t>((TUSB_DESC_STRING << 8) | (2 * count + 2));
        return g_string;
    }

    uint16_t tud_hid_get_report_cb(uint8_t, uint8_t, hid_report_type_t, uint8_t *, uint16_t) { return 0; }

    void tud_hid_set_report_cb(uint8_t, uint8_t, hid_report_type_t, uint8_t const *buffer, uint16_t size)
    {
        if (g_request_pending || size == 0)
            return;
        std::memset(g_request, 0, sizeof(g_request));
        std::memcpy(g_request, buffer, size < sizeof(g_request) ? size : sizeof(g_request));
        g_request_pending = true;
    }
}
