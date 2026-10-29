# Robot code

Use `src/main.cpp` as the single file for robot-specific ports, PID settings,
mechanism logic, controller macros, and autonomous code. Do not create a second
root-level `main.cpp` or split robot behavior into other files unless the user
requests it. Keep the required PROS/LemLib headers, libraries, and build files.

The current route is an A-button practice routine in driver control;
`autonomous()` is empty. Preserve that distinction unless asked to change it.
Historical sequences in the final comment are inactive references.
