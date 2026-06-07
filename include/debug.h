#ifndef DEBUG_H
#define DEBUG_H

#include <tusb.h>

#include <cstdio>
#include <cstring>
#include <cstdarg>

inline void cdcLogFlush()
{
#ifdef DEBUG
    if (!tud_cdc_connected())
        return;

    if (tud_cdc_write_flush() == 0)
    {
        return;
    }
#endif
}

inline void cdcLog(const char *format, ...)
{
    va_list args;
#ifdef DEBUG
    if (!tud_cdc_connected())
        return;

    char buf[256];
    va_start(args, format);
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    if (len <= 0)
        return;

    if (len > static_cast<int>(sizeof(buf) - 1))
        len = static_cast<int>(sizeof(buf) - 1);

    if (tud_cdc_write_available() < static_cast<uint32_t>(len))
        return;

    tud_cdc_write(buf, static_cast<size_t>(len));
#else
    (void)format;
    (void)args;
#endif
}

#endif // DEBUG_H