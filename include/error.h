#ifndef ERROR_H
#define ERROR_H

enum class Error
{
    // === No Error ===
    ERROR_NONE = 0,

    // === Software Errors (1-15) ===
    ERROR_VBUS_OUT_OF_RANGE = 1,

    // === Encoder Errors (2-7) ===
    ERROR_ENCODER_FAILED = 2,         // General encoder failure (e.g. no signal)
    ERROR_CPR_POLEPAIRS_MISMATCH = 3, // Detected mismatch between encoder CPR and motor pole pairs

    // === Hardware Faults - DRV8301 (16-19) ===
    ERROR_DRV_FAULT = 16,      // General nFAULT pin active
    ERROR_DRV_OTS = 17,        // Over-temperature shutdown
    ERROR_DRV_OC = 18,         // Overcurrent
    ERROR_DRV_GATE_FAULT = 19, // Gate drive fault
};

#endif // ERROR_H