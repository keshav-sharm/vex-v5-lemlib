# VEX V5 — LemLib starter

Fresh PROS V5 project with **PROS 4.2.2** and official **LemLib 0.5.6**, plus **liblvgl 9.2.0** for the example screen display.

`src/main.cpp` is the unmodified official LemLib v0.5.6 example. All ports,
dimensions, tracking wheel settings, PID values and tuning constants are the
upstream example defaults, not configuration for your robot. Configure these
before uploading/running on a robot.

## Open and build

Open this folder in VS Code with the PROS extension. Install its PROS CLI and
ARM toolchain when prompted, then use **PROS: Build** or `pros make` in the PROS
terminal. GitHub Desktop opens this same folder for reviewing and committing
changes; push commits to share updates.

The firmware libraries in `firmware/` are required vendor dependencies supplied
by PROS and LemLib, not project build outputs. Generated `bin/`, object files,
caches, and local editor configuration are ignored by Git.

## Official sources

- [LemLib getting started](https://lemlib.readthedocs.io/en/stable/tutorials/1_getting_started.html)
- [Exact starter source](https://github.com/LemLib/LemLib/blob/v0.5.6/src/main.cpp)
- [LemLib release](https://github.com/LemLib/LemLib/releases/tag/v0.5.6)
- [PROS release](https://github.com/purduesigbots/pros/releases/tag/4.2.2)
- [GitHub Desktop workflow](https://docs.github.com/en/desktop/adding-and-cloning-repositories/adding-an-existing-project-to-github-using-github-desktop)
