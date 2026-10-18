#ifndef USER_PREFS_STORE_H
#define USER_PREFS_STORE_H

#include <cstdint>

#include <persistence/config_store.h>

class UserPrefsStore
{
public:
    void load(PersistedConfig &config);

    bool save(const PersistedConfig &config, bool motor_running);

    void erase();

private:
    struct Record
    {
        uint32_t magic;
        uint8_t startup_sound_enabled;
        uint8_t idle_spring_enabled;
        uint8_t reserved[6];
        uint32_t crc32;
    };
    static_assert(sizeof(Record) == 16, "record must stay 16 bytes");

    const Record *findLast_(uint32_t &next_free_slot) const;
    static uint32_t crc_(const Record &record);
};

#endif // USER_PREFS_STORE_H
