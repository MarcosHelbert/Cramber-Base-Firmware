#include "config_store.h"

#include <cstring>

#include <libopencm3/stm32/flash.h>

PersistedConfig g_config;

namespace
{
    constexpr uint32_t kFlashSector = 11;
    constexpr uint32_t kStoreAddress = 0x080E0000;
    constexpr uint32_t kMagic = 0x43524253;
}

void ConfigStore::load()
{
    const StoredImage *stored = reinterpret_cast<const StoredImage *>(kStoreAddress);

    if (stored->magic != kMagic)
        return;
    if (stored->schema_version != kConfigSchemaVersion)
        return;
    if (stored->size != sizeof(PersistedConfig))
        return;
    if (stored->crc32 != computeCrc(*stored))
        return;

    g_config = stored->config;
}

void ConfigStore::save()
{
    StoredImage image{};
    image.magic = kMagic;
    image.schema_version = kConfigSchemaVersion;
    image.size = sizeof(PersistedConfig);
    image.config = g_config;
    image.crc32 = computeCrc(image);

    flash_unlock();
    flash_erase_sector(kFlashSector, FLASH_CR_PROGRAM_X32);

    const uint32_t *src = reinterpret_cast<const uint32_t *>(&image);
    uint32_t words = (sizeof(StoredImage) + 3) / 4;
    for (uint32_t i = 0; i < words; i++)
    {
        flash_program_word(kStoreAddress + (i * 4), src[i]);
    }

    flash_lock();
}

void ConfigStore::eraseStoredImage()
{
    flash_unlock();
    flash_erase_sector(kFlashSector, FLASH_CR_PROGRAM_X32);
    flash_lock();
}

uint32_t ConfigStore::crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++)
        {
            uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

uint32_t ConfigStore::computeCrc(const StoredImage &image)
{
    const uint8_t *start = reinterpret_cast<const uint8_t *>(&image.config);
    uint32_t len = sizeof(PersistedConfig);
    return crc32(start, len);
}