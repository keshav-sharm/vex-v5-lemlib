#pragma once

#include <cstdint>

// Timings are measured from when each phase actually starts, in milliseconds.
struct PinPickupConfig {
    std::uint32_t raiseMs = 900;
    std::uint32_t upperIntakeMs = 500;
    std::uint32_t lowerMs = 350;
    std::uint32_t baseIntakeMs = 500;
    int wristUpPower = 127; // same direction as B
    int wristDownPower = -127; // same direction as Down
    int intakePower = -127; // same direction as R1, for both intake and claw rollers
};

struct PinPickupOutput {
    int wristPower = 0;
    int intakePower = 0;
    bool active = false;
};

// Called from the driver loop; no delays or separate motor-writing tasks.
class PinPickupMacro {
public:
    explicit PinPickupMacro(PinPickupConfig config = {}) : config_(config) {}

    PinPickupOutput update(std::uint32_t now, bool startPressed, bool cancelRequested) {
        // Cancellation takes priority over a simultaneous start press.
        if (cancelRequested) {
            phase_ = Phase::Idle;
            return {};
        }
        if (startPressed && phase_ == Phase::Idle) {
            phase_ = Phase::Raising;
            phaseStartedAt_ = now;
        }

        // Unsigned subtraction also works when the millisecond counter wraps.
        const std::uint32_t elapsed = now - phaseStartedAt_;
        switch (phase_) {
            case Phase::Raising:
                if (elapsed >= config_.raiseMs) enter(Phase::UpperIntake, now);
                break;
            case Phase::UpperIntake:
                if (elapsed >= config_.upperIntakeMs) enter(Phase::Lowering, now);
                break;
            case Phase::Lowering:
                if (elapsed >= config_.lowerMs) enter(Phase::BaseIntake, now);
                break;
            case Phase::BaseIntake:
                if (elapsed >= config_.baseIntakeMs) enter(Phase::Idle, now);
                break;
            case Phase::Idle:
                break;
        }

        switch (phase_) {
            case Phase::Raising:
                return {config_.wristUpPower, 0, true};
            case Phase::UpperIntake:
            case Phase::BaseIntake:
                return {0, config_.intakePower, true};
            case Phase::Lowering:
                return {config_.wristDownPower, config_.intakePower, true};
            case Phase::Idle:
                return {};
        }
        return {};
    }

private:
    enum class Phase { Idle, Raising, UpperIntake, Lowering, BaseIntake };

    void enter(Phase phase, std::uint32_t now) {
        phase_ = phase;
        phaseStartedAt_ = now;
    }

    PinPickupConfig config_;
    Phase phase_ = Phase::Idle;
    std::uint32_t phaseStartedAt_ = 0;
};
