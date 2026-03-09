# YAMS (Yet Another Mechanism System)

This project uses YAMS (Yet Another Mechanism System) for mechanism abstractions (Arm, Elevator, Pivot) and SmartMotorController wrappers.

Quick links
-----------
- YAMS GitHub: https://github.com/Yet-Another-Software-Suite/YAMS
- YAMS docs / GitBook: https://yagsl.gitbook.io/yams/documentation
- YAMS vendordep JSON (for WPILib Manage Vendor Libraries):
  - https://yet-another-software-suite.github.io/YAMS/yams.json
- Bronc Botz (team / mentors): https://github.com/BroncBotz3481
- YASS / YAMS tutorial video (example): https://www.youtube.com/watch?v=BscPDiiYzrk

Command-based best-practice references
--------------------------------------
- BoVLB: https://bovlb.github.io/frc-tips/commands/best-practices.html
- WPILib docs (Organizing Command-Based projects): https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html
- ChiefDelphi discussion: https://www.chiefdelphi.com/t/command-based-best-practices-for-2025-community-feedback/465602

Why we reference these
----------------------
- YAMS provides the `Arm`, `ArmConfig`, and `SmartMotorController` abstractions used by the hood subsystem.
- YAMS includes simulation helpers (`simIterate()`), telemetry that integrates with AdvantageKit, and vendor wrapper implementations (e.g., `TalonFXWrapper`) that we use in this codebase.
- The command-based best-practice guides (BoVLB / WPILib / ChiefDelphi) describe the "command factory + triggers" pattern used across this repo (instance factories for single-subsystem commands, system-level factories for multi-subsystem orchestration, and use of `Trigger` for boolean state).

How to install YAMS via WPILib vendordep
----------------------------------------
1. In VS Code, open the Command Palette (Ctrl+Shift+P).
2. Select: `WPILib: Manage Vendor Libraries` -> `Install new library (online or offline)` -> `Online`.
3. Paste the vendordep URL:

   https://yet-another-software-suite.github.io/YAMS/yams.json

4. Finish the install — the YAMS vendordep will be added to your project.

License note
------------
YAMS is licensed under LGPL-3.0. Review the YAMS LICENSE before redistributing modified copies.

Suggested repo additions
------------------------
- Link this file into your project README or developer onboarding docs so new contributors know where to find YAMS and the recommended command-based patterns.
- Consider adding AdvantageKit logging entries for mechanism setpoints and measured angles to aid debugging (YAMS supports AdvantageKit integration).