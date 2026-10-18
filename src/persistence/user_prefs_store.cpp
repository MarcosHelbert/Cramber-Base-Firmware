#include "user_prefs_store.h"

#include <libopencm3/stm32/flash.h>

namespace
{
    constexpr uint32_t kFlashSector = 10;
    constexpr uint32_t kSectorAddress = 0x080C0000;
    constexpr uint32_t kSectorSize = 128 * 1024;
    constexpr uint32_t kMagic = 0x55505246;
    constexpr uint32_t kErased = 0xFFFFFFFFu;
}

uint32_t UserPrefsStore::crc_(const Record &record)
{
    const uint8_t *data = reinterpret_cast<const uint8_t *>(&record);
    const uint32_t len = sizeof(Record) - sizeof(record.crc32);

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

const UserPrefsStore::Record *UserPrefsStore::findLast_(uint32_t &next_free_slot) const
{
    const Record *records = reinterpret_cast<const Record *>(kSectorAddress);
    const uint32_t slots = kSectorSize / sizeof(Record);

    const Record *last = nullptr;
    next_free_slot = slots;
    for (uint32_t i = 0; i < slots; i++)
    {
        if (records[i].magic == kErased)
        {
            next_free_slot = i;
            break;
        }
        if (records[i].magic == kMagic && records[i].crc32 == crc_(records[i]))
            last = &records[i];
    }
    return last;
}

void UserPrefsStore::load(PersistedConfig &config)
{
    uint32_t next_free_slot;
    const Record *last = findLast_(next_free_slot);
    if (!last)
        return;

    config.startup_sound_enabled = last->startup_sound_enabled != 0;
    config.idle_spring_enabled = last->idle_spring_enabled != 0;
}

bool UserPrefsStore::save(const PersistedConfig &config, bool motor_running)
{
    uint32_t next_free_slot;
    const Record *last = findLast_(next_free_slot);

    Record record{};
    record.magic = kMagic;
    record.startup_sound_enabled = config.startup_sound_enabled ? 1 : 0;
    record.idle_spring_enabled = config.idle_spring_enabled ? 1 : 0;
    record.crc32 = crc_(record);

    if (last && last->startup_sound_enabled == record.startup_sound_enabled &&
        last->idle_spring_enabled == record.idle_spring_enabled)
        return true;

    if (next_free_slot >= kSectorSize / sizeof(Record))
    {
        if (motor_running)
            return false;
        erase();
        next_free_slot = 0;
    }

    const uint32_t address = kSectorAddress + next_free_slot * sizeof(Record);
    const uint32_t *words = reinterpret_cast<const uint32_t *>(&record);

    flash_unlock();
    flash_clear_status_flags();
    for (uint32_t i = 0; i < sizeof(Record) / 4; i++)
        flash_program_word(address + i * 4, words[i]);
    flash_lock();
    return true;
}

void UserPrefsStore::erase()
{
    flash_unlock();
    flash_clear_status_flags();
    flash_erase_sector(kFlashSector, FLASH_CR_PROGRAM_X32);
    flash_lock();
}
