#include <safety_supervisor.h>

#include <utils.h>

SafetySupervisor &SafetySupervisor::instance()
{
    static SafetySupervisor inst;
    return inst;
}

void SafetySupervisor::init(uint32_t now_ms)
{
    last_host_hid_ms_ = now_ms;
    last_encoder_update_ms_ = now_ms;
}

void SafetySupervisor::raiseFault(Error error, FaultLevel level, uint32_t now_ms)
{
    if (error != Error::ERROR_NONE)
    {
        if (active_error_ != error)
            fault_count_++;

        active_error_ = error;
    }

    if (static_cast<uint8_t>(level) > static_cast<uint8_t>(level))
        level = level;

    if (level == FaultLevel::LATCH_OFF)
        latch_off_ = true;

    last_fault_ms_ = now_ms;
}

void SafetySupervisor::clearNonLatchedFaults()
{
    if (latch_off_)
        return;

    active_error_ = Error::ERROR_NONE;
    level_ = FaultLevel::NONE;
}

bool SafetySupervisor::isLatchedOff() const
{
    return latch_off_;
}

Error SafetySupervisor::getActiveError() const
{
    return active_error_;
}

void SafetySupervisor::setCurrentDerate(float scale)
{
    const float s = clamp(scale, 0.0f, 1.0f);
    current_limit_scale_ = s;

    if (s < 0.999f)
    {
        if (level_ == FaultLevel::NONE || level_ == FaultLevel::WARNING)
            level_ = FaultLevel::DERATE;
    }
}

float SafetySupervisor::getCurrentLimit(float base_current) const
{
    const float scale = clamp(current_limit_scale_, 0.0f, 1.0f);
    return base_current * scale;
}

void SafetySupervisor::updateHostHeartbeat(uint32_t now_ms)
{
    last_host_hid_ms_ = now_ms;
}

void SafetySupervisor::markHostDisconnected()
{
    last_host_hid_ms_ = 0;
}

void SafetySupervisor::markEncoderUpdate(uint32_t now_ms)
{
    last_encoder_update_ms_ = now_ms;
}

uint32_t SafetySupervisor::getLastEncoderUpdateMs() const
{
    return last_encoder_update_ms_;
}

void SafetySupervisor::setLastFocIsrCycles(uint32_t cycles)
{
    last_foc_isr_cycles_ = cycles;
    if (cycles > max_foc_isr_cycles_)
        max_foc_isr_cycles_ = cycles;
}

uint32_t SafetySupervisor::getLastFocIsrCycles() const
{
    return last_foc_isr_cycles_;
}

uint32_t SafetySupervisor::getMaxFocIsrCycles() const
{
    return max_foc_isr_cycles_;
}