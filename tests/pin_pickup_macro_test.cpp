#include "pin_pickup_macro.hpp"

#include <cassert>
#include <cstdint>
#include <iostream>
#include <limits>

void expect(const PinPickupOutput& output, int wrist, int intake, bool active) {
    assert(output.wristPower == wrist);
    assert(output.intakePower == intake);
    assert(output.active == active);
}

void testSequenceAndIgnoredRetriggers() {
    PinPickupMacro macro;
    expect(macro.update(100, false, false), 0, 0, false);
    expect(macro.update(100, true, false), 127, 0, true);
    // A second press while busy must not reset or queue the sequence.
    expect(macro.update(600, true, false), 127, 0, true);
    expect(macro.update(999, false, false), 127, 0, true);
    expect(macro.update(1000, false, false), 0, -127, true);
    expect(macro.update(1499, false, false), 0, -127, true);
    expect(macro.update(1500, false, false), -127, -127, true);
    expect(macro.update(1849, false, false), -127, -127, true);
    expect(macro.update(1850, false, false), 0, -127, true);
    expect(macro.update(2349, false, false), 0, -127, true);
    expect(macro.update(2350, true, false), 0, 0, false);
    expect(macro.update(2360, false, false), 0, 0, false);
    expect(macro.update(2400, true, false), 127, 0, true);
}

void testCancelEveryPhaseAndRestart() {
    for (int phase = 0; phase < 4; ++phase) {
        PinPickupMacro macro;
        macro.update(0, true, false);
        if (phase >= 1) macro.update(900, false, false);
        if (phase >= 2) macro.update(1400, false, false);
        if (phase >= 3) macro.update(1750, false, false);
        // Cancel/manual override wins even when X is also pressed.
        expect(macro.update(1800, true, true), 0, 0, false);
        expect(macro.update(2000, false, false), 0, 0, false);
        expect(macro.update(2100, true, false), 127, 0, true);
    }
}

void testDelayedLoopDoesNotSkipMotorPhases() {
    PinPickupMacro macro;
    macro.update(0, true, false);
    // The next phase gets its own full duration after it actually begins.
    expect(macro.update(5000, false, false), 0, -127, true);
    expect(macro.update(5499, false, false), 0, -127, true);
    expect(macro.update(5500, false, false), -127, -127, true);
}

void testClockRollover() {
    PinPickupMacro macro;
    const std::uint32_t start = std::numeric_limits<std::uint32_t>::max() - 400;
    expect(macro.update(start, true, false), 127, 0, true);
    expect(macro.update(start + 899U, false, false), 127, 0, true);
    expect(macro.update(start + 900U, false, false), 0, -127, true);
}

int main() {
    testSequenceAndIgnoredRetriggers();
    testCancelEveryPhaseAndRestart();
    testDelayedLoopDoesNotSkipMotorPhases();
    testClockRollover();
    std::cout << "Pin pickup macro tests passed\n";
}
