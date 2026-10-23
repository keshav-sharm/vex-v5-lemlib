# 295W VEX V5 — LemLib

PROS 4.2.2 / LemLib 0.5.6 project using the teammate's supplied robot configuration and controller code as the base for `src/main.cpp`.

## A-button macro: one toggle pulse, forward retreat, turn

Start from the same position and wrist height as IMG_4824, with the toggle yellow.
The routine holds the existing lift and wrist positions, leaves the separate
intake motor stopped, and:

1. Targets a **3-inch backward push**, capped at **30 out of 127**, while running the claw at **+127** for **1400 ms**, then brakes the claw.
2. Waits for the push to finish, then pauses **3 seconds** with the claw stopped.
3. Drives **forward 4 inches** from that position. There is no second toggle pulse.
4. Turns to **180 degrees** relative to the heading zeroed before the forward drive, capped at **50 out of 127**, with a **3000 ms** timeout.

The video shows the toggle passing blue and continuing to rotate during the old
3-second pulses. The shorter pulse is an initial timing estimate, not a verified
blue stop. There is no color feedback in this routine. Test from the same yellow
start and adjust only `pros::delay(1400)` if the toggle stops short or goes past blue.

The push has a **3000 ms** timeout; contact can prevent it reaching its target. The forward retreat
is capped at **50 out of 127** with a **2000 ms** timeout. These starting power
and timing values need checking on the robot.
The supplied PID settling checks are disabled, so a movement may use its full timeout.

The steps are written directly inside the A-button `if` in `opcontrol()`.
`setPose(0, 0, 0)` before each movement makes its
distance relative to that movement's starting point and facing direction:
Y = -3 means backward 3 inches, and Y = 4 means forward 4 inches.
The practice macro resets the odometry coordinates; `autonomous()` is empty.

## Controller controls

- **X, pressed once:** starts wrist-up, DR4B-up, and claw outtake together with a gentle 100 ms power step. The DR4B uses **300 ms total** at +40 then +60 as an initial timed estimate toward a **3-inch rise**; this height is not calibrated. The wrist runs **1 second** at +40 / +80 / +40; the claw outtakes **2 seconds** at +60 / +100 / +60. Each mechanism brakes when its time ends. The drive is braked and the separate intake stopped during the macro. Test from the same lift/wrist starting position and tune the lift's `pros::delay(200)` after the initial 100 ms step to adjust its rise.
- **A, pressed once:** runs the push/outtake sequence directly in driver control. Holding A does not repeat it.
- **L1 / L2:** lift up / down.
- **R1 / R2:** intake and claw rollers inward / outward.
- **Up / Left:** wrist in the positive / negative direction.
- **Down:** the teammate's unfinished wrist-position placeholder.
- **Right, pressed once:** the teammate's 3-second claw/wrist/lift macro.
- **B, pressed once:** the teammate's lift/wrist position command.
- **Y, pressed once:** the teammate's separate 3-second forward-drive/lift/wrist/claw macro, including full-power forward drive.

Use the **Brain Stop button** to stop the program during a blocking test. Manual
driver inputs are not processed during the A, X, Right, or Y blocking sequences.
The previous pickup macro is no longer connected to this replacement main file;
its header/test files are retained locally as reference.

The supplied missing final brace was restored, and out-of-range `.move()` powers
in the Right/Y macros were limited to the valid -127..127 range. Motor ports,
position targets, and the other supplied mappings are retained.

The teammate's current PID gains are lateral **10 / 0 / 10**, slew **2.5**;
angular **2 / 0 / 10**, slew **0**. Both have zeroed error-range exit settings.

## Open and build

Open this folder in VS Code with the PROS extension. Install its PROS CLI and
ARM toolchain when prompted, then use **PROS: Build** or `pros make` in the PROS
terminal. GitHub Desktop opens this same folder for reviewing and committing
changes; push commits to share updates.

Practice uploads default to **slot 4** and show the program screen without
running it. This project uses a self-contained binary (`USE_PACKAGE:=0`) so
uploads do not replace a shared PROS library used by other programs. Full
uploads are larger than hot/cold updates and may take longer over the radio.

The installed GCC 14 toolchain was verified with
`make -j2 CXX_STANDARD=gnu++23 C_STANDARD=gnu17`. If a build reports an
unsupported `gnu++26` option, use those overrides with `make` in the PROS
terminal, or update the PROS toolchain to one that supports the project defaults.

The firmware libraries in `firmware/` are required vendor dependencies supplied
by PROS and LemLib, not project build outputs. Generated `bin/`, object files,
caches, and local editor configuration are ignored by Git.

## Official sources

- [LemLib getting started](https://lemlib.readthedocs.io/en/stable/tutorials/1_getting_started.html)
- [LemLib lateral motions](https://lemlib.readthedocs.io/en/stable/tutorials/6_lateral_motion.html)
- [LemLib angular motions](https://lemlib.readthedocs.io/en/stable/tutorials/5_angular_motion.html)
- [LemLib exit conditions](https://lemlib.readthedocs.io/en/stable/tutorials/4_pid_tuning.html#exit-conditions)
- [Exact starter source](https://github.com/LemLib/LemLib/blob/v0.5.6/src/main.cpp)
- [LemLib release](https://github.com/LemLib/LemLib/releases/tag/v0.5.6)
- [PROS release](https://github.com/purduesigbots/pros/releases/tag/4.2.2)
- [GitHub Desktop workflow](https://docs.github.com/en/desktop/adding-and-cloning-repositories/adding-an-existing-project-to-github-using-github-desktop)
