#ifndef FFB_MANAGER_H
#define FFB_MANAGER_H

#include <array>
#include <cstdint>

#include <ffb_protocol.h>
#include <persistence/config_store.h>
#include "ffb_engine.h"

class FFBManager
{
public:
    explicit FFBManager(const PersistedConfig &config);

    void startFFB()
    {
        ffbActive_ = true;
        devicePaused_ = false;
    }
    void stopFFB() { ffbActive_ = false; }
    void resetFFB();

    bool isFFBActive() const { return ffbActive_ && !devicePaused_; }
    bool isDevicePaused() const { return devicePaused_; }

    bool isGameDriving(uint32_t nowMs) const
    {
        constexpr uint32_t kHostSilentTimeoutMs = 5000;
        return ffbActive_ && !devicePaused_ && activeMask_ != 0 &&
               (nowMs - lastHostReportMs_) < kHostSilentTimeoutMs;
    }
    uint8_t getGain() const { return gain_; }

    const std::array<Effect, MAX_EFFECTS> &getEffects() const { return effects_; }

    void hidSet(uint8_t report_id, uint8_t const *buffer, uint16_t bufsize);
    uint16_t hidGet(uint8_t report_id, uint8_t *buffer);

    float getTorque(float positionRad, float velocityRadS, float accelerationRadS2, uint32_t nowMs);

private:
    bool isValidIndex_(uint8_t index) const { return index > 0 && index <= effects_.size(); }
    void updateBlockLoad_(uint8_t index, uint8_t status);

    static void recompute_(Effect &e);

    void sendStatusReport_();
    void setActive_(uint8_t idx, bool active);
    void setEffect_(SetEffect_Data *d);
    void setEnvelope_(SetEnvelope_Data *d);
    void setCondition_(SetCondition_Data *d);
    void setPeriodic_(SetPeriodic_Data *d);
    void setConstantForce_(SetConstantForce_Data *d);
    void setRamp_(SetRamp_Data *d);
    void effectOperation_(EffectOperation_Data *d);
    void blockFree_(uint8_t idx);
    void deviceControl_(uint8_t command);
    void deviceGain_(uint8_t gain);
    void createNewEffect_(NewEffect_Data *d);

    int32_t findFreeEffectIndex_(uint8_t type) const;

private:
    const PersistedConfig &config_;

    bool ffbActive_ = false;
    bool devicePaused_ = false;
    uint8_t gain_ = 255;

    FFBEngine engine_;
    std::array<Effect, MAX_EFFECTS> effects_{};

    uint16_t usedEffects_ = 0;
    uint32_t activeMask_ = 0;
    volatile uint32_t lastHostReportMs_ = 0;

    BlockLoad_Data blockLoad_{};
    PIDPool_Data pool_{};
};

#endif // FFB_MANAGER_H