# ORCABot2026 - Code Review & Competition Readiness Plan
**Reviewed: 2026-02-22 | Target: First competition in 1-2 weeks | Estimated: 20-30 hours of work**

---

## Current State Summary

The robot code has a solid command-based foundation with YAGSL swerve drive, PID-controlled shooter and hood, and all subsystems structurally in place. However, there are **10 bugs** that need fixing, **no competition button bindings** (only test bindings are active), **a non-functional vision system**, and **incomplete autonomous routines**. The code needs focused work to be competition-ready.

---

## BUGS TO FIX (All should be fixed before competition)

### Bug 1: RunConveyorCommand missing kicker requirement
- **File:** `commands/RunConveyorCommand.java` line 30
- **Problem:** Command uses KickerSubsystem but doesn't declare it as a requirement. This means the command scheduler could run two commands on the kicker simultaneously, causing erratic behavior.
- **Fix:** Add `addRequirements(kickerSubsystem)` alongside the conveyor requirement.

### Bug 2: Kicker telemetry publishes to wrong NetworkTable key
- **File:** `subsystems/KickerSubsystem.java` line 70
- **Problem:** Kicker current publishes to `Conveyor.kCurrentAmps` instead of `Kicker.kCurrentAmps`. Dashboard shows wrong data.
- **Fix:** Change to `NetworkTableNames.Kicker.kCurrentAmps`.

### Bug 3: Pigeon2 angular velocity suppliers all read the same axis
- **File:** `subsystems/SwerveSubsystem.java` lines 78-80
- **Problem:** Roll, pitch, and yaw suppliers all call `getAngularVelocityXDevice()`. Should be different axes (X/Y/Z).
- **Fix:** Use `getAngularVelocityYDevice()` for roll and `getAngularVelocityZDevice()` for pitch (verify axis mapping based on Pigeon2 mounting).
- **Also:** `subsystems/VisionSubsystem.java` lines 163-170 has the same issue - angular velocity array uses `[0]` for all three axes instead of `[0]`, `[1]`, `[2]`.

### Bug 4: SwerveSubsystem.getTranslationToFieldElement() broken switch
- **File:** `subsystems/SwerveSubsystem.java` lines 541-552
- **Problem:** Each switch case computes a `.minus()` result but doesn't `return` it. Values are calculated and thrown away. All cases fall through to `return null`.
- **Fix:** Add `return` before each `.minus()` call.

### Bug 5: SlowHoodMove command never applies the target
- **File:** `commands/SlowHoodMove.java` line 50
- **Problem:** `m_hoodSubsystem.setHoodTarget(targetPosition)` is commented out. Command calculates a target every cycle but never sends it to the subsystem.
- **Fix:** Uncomment the line.

### Bug 6: Climber position limits both set to 0
- **File:** `Constants.java` lines 80-81, `commands/RunClimberCommand.java` lines 56-59
- **Problem:** `kClimberMaxPosition` and `kClimberMinPosition` are both 0, and the limit checks in RunClimberCommand are commented out. No safety stops.
- **Fix:** Run climber manually, read encoder values at full extend/retract from NetworkTables, set actual limits, uncomment the checks.
- **Action needed from team:** Measure the physical limits on the robot.

### Bug 7: Limelight names mismatch
- **File:** `Constants.java` lines 87-88
- **Problem:** CLAUDE.md says `"limelight-front"` / `"limelight-back"` but code says `"limelight-left"` / `"limelight-right"`. One must be wrong.
- **Action needed from team:** Check the actual Limelight web interface to see configured hostnames, then update Constants.java to match.

### Bug 8: LaserCan code needs removal
- **Files:** `subsystems/VisionSubsystem.java`, `Robot.java`
- **Problem:** LaserCan hardware is being removed from robot but code still references it. `getLidarMeasurement()` would crash with NullPointerException if called.
- **Fix:** Remove imports, fields, and methods related to LaserCan. Remove `CanBridge.runTCP()` from Robot.java (verify no other grapple hardware first). Potentially remove `vendordeps/libgrapplefrc2026.json`.

### Bug 9: IntakeDeployMotor config type mismatch
- **File:** `Configs.java` line 84 vs `subsystems/IntakeSubsystem.java` line 61
- **Problem:** Config uses `SparkFlexConfig` but the actual motor is a NEO 550 on a SparkMax. The config type should match the controller.
- **Fix:** Change `Configs.java` to use `SparkMaxConfig` instead of `SparkFlexConfig`.

### Bug 10: VisionSubsystem is completely non-functional
- **File:** `subsystems/VisionSubsystem.java` lines 112-148
- **Problem:** The `bestLimelight` variable is initialized to -1 and never updated. The fitness scoring logic is incomplete (all values are 0). The condition `if (updatePose && bestLimelight != -1)` can never pass. Vision poses are **never applied** to odometry.
- **Fix:** Requires rewrite of `updateRobotPosition()` (see Tier 2 below).

---

## TIER 1: MUST HAVE FOR COMPETITION (~12 hours)

### 1. Competition Button Bindings (~2.5 hours)
**Current state:** Only `configureTestBindings()` is active, with shooter speed increment buttons. No intake deploy, no shooting sequence, no climber controls.

**Proposed layout:**

**Driver (Xbox Controller, Port 0):**
| Control | Action |
|---------|--------|
| Left Stick | Drive translation |
| Right Stick | Drive rotation |
| Left Trigger (hold) | Intake sequence: deploy intake + run rollers + run conveyor inward |
| Right Trigger (hold) | Feed/shoot: run conveyor + kicker into spinning shooter |
| Right Bumper | Preset: Hub shot (set shooter RPM + hood angle) |
| Left Bumper | Preset: Close/dump shot |
| A | Stop shooter |
| B | Zero gyro |
| Start (hold) | Climber up |
| Back (hold) | Climber down |
| D-Pad Up/Down | Manual conveyor forward/reverse (unjam) |

**Operator (Button Board, Port 1):**
| Button | Action |
|--------|--------|
| 1 | Shooter preset: Close shot (Low RPM + low hood) |
| 2 | Shooter preset: Medium shot |
| 3 | Shooter preset: Far shot (High RPM + high hood) |
| 4 | Stop shooter |
| 5 | Hood angle fine adjust down |
| 6 | Hood angle fine adjust up |
| 7 | Manual conveyor forward (unjam) |
| 8 | Manual conveyor reverse (unjam) |
| 9 | Emergency stop all mechanisms |
| 10 | Toggle intake deploy (manual override) |

**Team discussion needed:** Review this layout with drivers. Adjust based on driver preference.

### 2. Automated Shooting Sequence Command (~2.5 hours)
**Problem:** Currently there's no coordinated shooting - the driver has to manually set shooter speed, manually set hood angle, and manually trigger the conveyor/kicker in the right order.

**Solution:** New `ShootSequenceCommand` that:
1. Sets shooter RPM and hood angle simultaneously
2. Waits until both reach their targets
3. Runs conveyor + kicker to feed fuel

Needs new methods: `ShooterSubsystem.isAtTarget()` and `HoodSubsystem.isAtTarget()` to check if mechanisms are at setpoint.

Create preset shooting configs (close/medium/far) tied to button board buttons and driver bumpers.

### 3. Re-enable Intake Deploy PID (~1 hour)
**Problem:** `IntakeSubsystem.java` line 213 has `setPIDAngle()` commented out. The intake has PID configured in Configs.java but never actually uses it. Deploy/retract is not position-controlled.

**Fix:** Uncomment, verify position limits match physical hardware, wire into trigger binding (hold trigger = deploy + spin rollers, release = retract).

### 4. Fix Named Commands for PathPlanner (~1 hour)
**Problem:** `RunConveyorCommand` has `isFinished()` returning `false` (never finishes). When used in sequential auto groups, this blocks the entire auto routine from progressing. Several autonomous routines are broken because of this.

**Fix:** Change named commands for "Run Conveyor" and "Stop Conveyor" to use `Commands.runOnce()` that just sets velocities:
```java
// Instead of new RunConveyorCommand(...) which never finishes:
NamedCommands.registerCommand("Run Conveyor", Commands.runOnce(() -> {
    conveyorSubsystem.setConveyorVelocity(500);
    kickerSubsystem.setKickerVelocity(4000);
}));
```

### 5. Working Autonomous Routines (~3 hours)
**Goal:** 3 reliable autos that actually score fuel in the 20-second auto period.

1. **"Score and Taxi"** - Shoot preloaded fuel, drive away. Simple, reliable fallback.
2. **"Score and Collect"** - Shoot preload, drive to depot/outpost, intake more fuel, return and shoot again.
3. **"Score and Climb"** - Shoot preload, drive to tower, climb for endgame points early.

All autos need the fixed named commands from item 4 above, plus the new shooting sequence registered as a named command.

---

## TIER 2: HIGH VALUE - Competitive Advantage (~10 hours)

### 6. Fix and Enable Vision System (~3.5 hours)
**Why it matters:** Vision-corrected odometry is critical for accurate autonomous paths and could enable features like auto-aim. Without it, the robot relies purely on wheel odometry which drifts over time.

**Work needed:**
- Rewrite `VisionSubsystem.updateRobotPosition()` with working logic
- Enable VisionSubsystem in RobotContainer (currently commented out)
- Add proper standard deviations for vision measurements in odometry fusion
- Test with AprilTags on practice field

### 7. Beam Break Sensor Integration (~2.5 hours)
**Status:** Sensor physically installed between kicker and shooter, needs to be wired.

**What it enables:**
- Know when fuel is loaded and ready to shoot
- Automated feed-until-empty for rapid fire
- Prevents feeding into a shooter that isn't spun up

**Implementation:** Create BeamBreakSensor wrapper, integrate into shooting sequence, publish to NetworkTables. **Important:** Design so shooting still works time-based if beam break isn't available.

### 8. Improved Autonomous Routines (~3 hours)
With vision online, create more aggressive multi-ball autos:
- 3-Ball Left Depot Auto
- 3-Ball Right Outpost Auto
- 5-Ball Center Auto (ambitious)

---

## TIER 3: NICE TO HAVE (if time permits)

### 9. Hub Auto-Aim (~2 hours)
Hold a button to automatically rotate the robot to face the hub while driver controls translation. Uses field element position tracking from SwerveSubsystem.

### 10. Distance-Based Shooter Auto-Tuning (~1.5 hours)
Automatically set shooter RPM and hood angle based on distance to hub using interpolation tables. Requires empirical data from testing shots at multiple distances.

### 11. Shift Timer Display (~1 hour)
Track the 25-second hub shift schedule during teleop. Display which hub is currently active on the dashboard.

### 12. Code Cleanup (~0.5 hours)
Remove ExampleSubsystem/Command, ShooterSelfFeedCommand stub, and dead commented-out code throughout the project.

---

## Suggested Session Schedule

| Session | Hours | Work Items |
|---------|-------|------------|
| 1 | ~4 hrs | Bug fixes (all 10) + start competition bindings |
| 2 | ~4 hrs | Finish bindings + shooting sequence command + intake PID |
| 3 | ~4 hrs | Fix named commands + auto routines + **test on robot** |
| 4 | ~4 hrs | Vision system fix + test with AprilTags |
| 5 | ~4 hrs | Beam break integration (when wired) + improved autos |
| 6 | ~4 hrs | Tier 3 items + competition prep tuning |
| Pre-comp | Bring laptop | Field testing, preset tuning, path adjustments |

---

## Key Risks

| Risk | Mitigation |
|------|-----------|
| Beam break not wired in time | Shooting sequence falls back to time-based feeding |
| Vision estimates noisy | Start with conservative trust levels, tighten after validation |
| Auto paths don't match field | Always have "Score and Taxi" as failsafe auto |
| Intake PID values wrong | Output already clamped to ±0.5 in config, test on blocks first |
| Named command changes break existing autos | Test all autos after changing named command registrations |
