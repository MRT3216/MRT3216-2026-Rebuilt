# Assistant Profile — MRT3216 Command-Based + YAMS Conventions

Purpose
-------
This file captures the distilled programming conventions, command-based patterns, and YAMS-specific best practices used in this repository. Keep this file in the repo so the assistant and team members can quickly rehydrate the same expectations on any machine.

How to use
----------
- Copy the "Starter prompt" section below into an assistant session to resume the same conventions.
- Refer to the "Quick snippets" section when asking the assistant to make edits or run checks.

Design philosophy (short)
-------------------------
- Prefer YAMS-first, command-returning APIs for mechanisms. Expose methods like `setVelocity(AngularVelocity)` and `setVelocity(Supplier<AngularVelocity>)` which return WPILib Commands.
- Keep imperative helpers private and short (e.g., `applySetpoint(...)`) and provide one-shot helpers for sequence use (e.g., `stopNow()`).
- Two-stop semantics:
  - `stopHold()`: closed-loop zero-speed command that holds while scheduled (use as default command or when an operator wants to hold idle state).
  - `stopNow()`: one-shot, imperative stop used in sequences to quickly apply zero and continue.
- Use `followTarget(Supplier<AngularVelocity>)` when a YAMS supplier-backed command doesn't re-evaluate live; implement an imperatively re-applier that writes the supplier value each scheduler tick for tuning workflows.
- Keep subsystem ownership clear: commands that alter persistent behavior should generally be created by systems or containers (caller-owns pattern). Short, localized runOnce actions are acceptable from many places.

Naming & observability
----------------------
- Use `withName("MeaningfulName")` on composed commands (e.g., `StartShooting`, `FeedAndShoot`, `AimAndShoot`) to improve Scheduler logs and AdvantageKit traces.
- Publish minimal dashboard keys for quick debugging (optional): `Shooter/RequestedRPM`, `Shooter/AppliedRPM`, `Shooter/TurretErrorDeg`, `Shooter/HoodErrorDeg`.
- Rely on YAMS + AdvantageKit telemetry for detailed mechanism state; add a few NetworkTables values for operator-facing widgets.

Operator ergonomics
-------------------
- Bump commands should be one-shot and not require subsystems. That lets the operator adjust targets while a feed pipeline is running without interrupting it.
- Trigger-driven shooting: prefer `whileTrue(...)` bindings to run a composed shooting pipeline while the operator holds a trigger and automatically cancel on release.

Starter prompt (paste to assistant)
-----------------------------------
Use this exact block to rehydrate assistant behavior. It summarizes the repo's conventions and desired behavior for edits.

"Project assistant profile: MRT3216 repo. Use YAMS-first command-returning APIs. Prefer `setVelocity(...)` returns a Command. Use `stopHold()` for default closed-loop zero and `stopNow()` for one-shot imperative stops used inside sequences. Provide `followTarget(Supplier)` re-applier for live tuning when supplier-backed YAMS commands do not re-evaluate. Name composed commands with `withName(...)`. Bump commands must be one-shot and not require subsystems. When making edits, update the `docs/assistant-profile.md` if conventions change and run `./gradlew.bat build` to validate. If you are unsure, ask for clarification."

Quick snippets
--------------
- Add a named composed command:
  - `return Commands.parallel(a, b).withName("MeaningfulName");`
- Create a one-shot bump that doesn't require subsystems:
  - `return Commands.runOnce(() -> { /* update atomic ref */ }).withName("BumpFlywheelSys");`
- Default command to hold zero:
  - `flywheel.setDefaultCommand(flywheel.stopHold());`
- Follow-target re-applier:
  - `return Commands.run(() -> applySetpoint(supplier.get()), this).withName("FlywheelFollowTarget");`

Privacy & secrets
-----------------
- Do not commit secrets (API keys, service tokens). If a conversation contains a secret, redact it before committing.
- This file is safe to commit; remove or redact any sensitive lines before adding additional chat transcripts.

Commit policy for assistant backups
-----------------------------------
- Prefer a single ongoing file `docs/assistant-history.md` for day-to-day append-only backups, or timestamped files for archives. Keep backups private if needed.

Append policy (how I will append)
---------------------------------
- When requested, the assistant will append the verbatim user+assistant transcript (or the user/assistant subset) to `docs/assistant-history.md` under a timestamped header. It will never push unredacted secrets unless you explicitly instruct it to.

Contact & maintenance
---------------------
- If you change the repo's conventions (e.g., decide to gate feeding only when at-speed), update this profile so the assistant uses the new rules.

---

*Generated on 2026-02-27 — commit pushed to `mechanisms` by automation on user request.*
