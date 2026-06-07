#ifndef CAN_H
#define CAN_H

#include <cstdint>

class CAN
{
public:
    CAN();

    void init(uint32_t bitrate_bps = 1000000);

private:
};

#endif // CAN_H