# Execution Plan Format

This document defines the structured format used when an **advanced planning model** (e.g., Claude Opus) creates a plan for a **cheaper execution model** (e.g., Claude Sonnet, GPT-4o, Copilot Chat) to implement.

## How to use

1. **Planning session** (expensive model): User describes the goal. Planner reads codebase, makes decisions, and outputs a plan using the template below.
2. **Execution session** (cheaper model): User pastes the plan into a new chat. The executor follows it step-by-step, making no judgment calls — just mechanical edits.
3. **Verification**: The executor runs the build/test commands listed in the plan and reports results.

### What the executor should do
- Follow each step **literally** — the planner has already made all design decisions.
- If a step fails (compile error, test failure), report the error and stop. Don't improvise fixes.
- Read the referenced context files (`docs/assistant/profile.md`, `docs/assistant/history.md`) if the plan says to, but don't re-derive decisions already made.

### What the executor should NOT do
- Change the architectural approach.
- Skip steps or reorder them.
- Add features not in the plan.
- Make "improvements" to code outside the plan scope.

---

## Plan Template

Copy everything below this line into a new chat with the execution model.

---

````markdown
# Execution Plan: [TITLE]

**Created by**: [planner model name] on [date]
**Branch**: [working branch]
**Repository**: MRT3216/MRT3216-2026-Rebuilt (FRC 2026 robot code — Java, WPILib, YAMS, AdvantageKit)

## Context

> Read `docs/assistant/profile.md` for project conventions and `docs/assistant/history.md` for recent changes.
> [Any additional context the executor needs — e.g., "The turret uses YAMS Pivot with SparkMax, 27:1 gearing"]

## Goal

[One paragraph: what this plan accomplishes and why]

## Pre-conditions

- [ ] On the correct working branch
- [ ] `.\gradlew compileJava` passes before starting
- [ ] [Any other prerequisites]

## Steps

### Step 1: [Short title]

**File**: `src/main/java/frc/robot/path/to/File.java`
**Action**: [edit | create | delete | rename]

[Precise description of what to change. For edits, include the EXACT old text and new text:]

**Find this:**
```java
// 3-5 lines of context before
THE LINE(S) TO CHANGE
// 3-5 lines of context after
```

**Replace with:**
```java
// 3-5 lines of context before
THE NEW LINE(S)
// 3-5 lines of context after
```

**Why**: [One sentence — helps executor understand intent if they hit an edge case]

---

### Step 2: [Short title]

[Same format...]

---

## Verification

After all steps, run these commands and confirm they pass:

```powershell
cd c:\Users\danla\Documents\GitHub\MRT3216-2026-Rebuilt
.\gradlew compileJava
```

Expected: `BUILD SUCCESSFUL`

[Optional: specific things to check in sim, AdvantageScope, etc.]

## Commit

```powershell
git add -A
git commit -m "[type]: [description]"
git push
```

## Rollback

If something goes wrong and you need to undo everything:

```powershell
git checkout -- .
```

## Notes for executor

- [Any gotchas, e.g., "Spotless may reformat — that's fine, don't fight it"]
- [Edge cases, e.g., "If import X doesn't resolve, the vendordep may need updating"]
````

---

## Example: Minimal plan

````markdown
# Execution Plan: Add kD to turret PID

**Created by**: Claude Opus on 2026-03-24
**Branch**: Claude
**Repository**: MRT3216/MRT3216-2026-Rebuilt

## Context

> Read `docs/assistant/profile.md` for project conventions.
> The turret uses YAMS Pivot with SparkMax. PID units are Volts per mechanism rotation (not degrees). See the "SparkMax PID/FF unit system" gotcha in profile.md.

## Goal

Add derivative gain (kD) to the turret to reduce overshoot observed during testing. kD=1.0 is a conservative starting value.

## Pre-conditions

- [ ] On the correct working branch
- [ ] `.\gradlew compileJava` passes

## Steps

### Step 1: Set turret kD

**File**: `src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java`
**Action**: edit

**Find this:**
```java
        public static final double kP = 100.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
```

**Replace with:**
```java
        public static final double kP = 100.0;
        public static final double kI = 0.0;
        public static final double kD = 1.0;
```

**Why**: Add derivative gain to dampen turret overshoot.

## Verification

```powershell
.\gradlew compileJava
```

Expected: `BUILD SUCCESSFUL`

## Commit

```powershell
git add -A
git commit -m "tune: add kD=1.0 to turret PID for overshoot damping"
git push
```
````
