# ORCABot2026 - Code Review & Competition Readiness Plan
**Reviewed: 2026-02-22 | Target: First competition in 1-2 weeks | Estimated: 20-30 hours of work**

---

## Current State Summary

All individual subsystems are working and tested on the robot: swerve drive, shooter (RPM control), hood (angle control), intake (rollers + deploy), conveyor, kicker, and climber. The test bindings let us manually adjust each mechanism in isolation.

**What's missing:** Nothing is *coordinated*. A driver can't press one button to shoot - they'd have to manually set RPM, set hood angle, then trigger the conveyor/kicker. There are no gameplay sequences, no shooting presets, and most critically: **we don't yet know what RPM + hood angle combinations actually score at different distances.**

**The path to competition-ready has two parallel tracks:**
1. **On-robot characterization** - Discover the right shooting parameters through testing (team task)
2. **Code coordination** - Build the sequences, bindings, and autos that use those parameters (coding task)

---

## PHASE 0: SHOOTING CHARACTERIZATION (On-Robot Testing)
*This is the #1 blocker. Everything else depends on knowing what values work.*

### What we need to figure out
A lookup table of **distance → (shooter RPM, hood angle)** that reliably scores in the Hub. We need at minimum 3-4 data points:

| Distance from Hub | Shooter RPM | Hood Angle (degrees) | Notes |
|-------------------|-------------|---------------------|-------|
| Point blank (~2 ft) | ? | ? | Dump shot |
| Close (~5 ft) | ? | ? | |
| Medium (~10 ft) | ? | ? | Primary shooting distance |
| Far (~15+ ft) | ? | ? | Cross-field shot |

### How to collect this data
The current test bindings are almost good enough for this. We need:

1. **Improve characterization controls** (small code change)
   - Current test bindings let you increase/decrease RPM and hood angle
   - Need: Display current RPM + hood angle + distance to hub clearly on dashboard/Shuffleboard
   - Need: A "feed one ball" button (run kicker for ~0.5 sec then stop) so you can take single shots

2. **Testing procedure** (team activity, ~1-2 hours on robot)
   - Place robot at known distances from Hub
   - Dial in RPM and hood angle until balls consistently go in
   - Record the values in the table above
   - Test at 3-4 distances minimum
   - **Tip:** Start with medium distance, find values that work, then adjust from there

3. **Record results into Constants.java**
   - These become the presets that drive everything else (competition bindings, auto routines, etc.)

### Dependencies
- Bug fix #9 (intake deploy config mismatch) should be done first to avoid startup issues
- Bug fix #1 (conveyor command requirement) should be done so the kicker doesn't conflict during testing

---

## PHASE 1: COORDINATION - Making the Robot Playable (~8-10 hours)
*Build sequences that let a driver actually play the game. Can start in parallel with Phase 0.*

### Step 1: Bug fixes that affect testing (~2 hours)
Fix the bugs that could cause issues during characterization and gameplay. See full bug list below.

**Priority order for fixes:**
1. Bug #1 (ConveyorCommand missing kicker requirement) - causes kicker conflicts
2. Bug #9 (IntakeDeployMotor config mismatch) - could crash on startup
3. Bug #2 (Kicker NT key wrong) - affects telemetry reading during characterization
4. Bug #6 (Climber limits) - safety concern
5. Bug #8 (LaserCan removal) - cleanup
6. Bugs #3, #4, #5, #7 - lower priority, fix when convenient

### Step 2: Shooting sequence command (~2 hours)
**New command: `ShootSequenceCommand`**
- Input: shooterRPM, hoodAngleDegrees (from the characterization table)
- Behavior: Set shooter + hood in parallel → wait until at target → feed with conveyor + kicker
- Needs: `ShooterSubsystem.isAtTarget(toleranceRPM)` and `HoodSubsystem.isAtTarget(toleranceDeg)` methods
- Create preset factories: `closeShot()`, `mediumShot()`, `farShot()` using characterized values

### Step 3: Intake sequence (~1 hour)
**Coordinated intake:** One trigger does everything:
- Deploy intake mechanism
- Run intake rollers
- Run conveyor inward to stage fuel
- On release: retract intake, stop rollers

Requires re-enabling intake deploy PID (Bug #4 in the intake - currently commented out).

### Step 4: Competition button bindings (~2 hours)
Wire the sequences into driver-friendly controls:

**Driver (Xbox, Port 0):**
| Control | Action |
|---------|--------|
| Left Stick | Drive |
| Right Stick | Rotate |
| Left Trigger (hold) | Full intake sequence (deploy + rollers + conveyor) |
| Right Trigger (hold) | Feed into shooter (conveyor + kicker) |
| Right Bumper | Preset: Medium shot (most common distance) |
| Left Bumper | Preset: Close/dump shot |
| A | Stop shooter |
| B | Zero gyro |
| Start (hold) | Climber up |
| Back (hold) | Climber down |
| D-Pad Up/Down | Manual conveyor fwd/rev (unjam) |

**Operator (Button Board, Port 1):**
| Button | Action |
|--------|--------|
| 1 | Shooting preset: Close |
| 2 | Shooting preset: Medium |
| 3 | Shooting preset: Far |
| 4 | Stop shooter |
| 5-6 | Hood fine adjust down/up |
| 7-8 | Manual conveyor fwd/rev (unjam) |
| 9 | Emergency stop all |
| 10 | Manual intake deploy toggle |

### Step 5: Basic autonomous routines (~2 hours)
Using characterized shooting values:
1. **"Score and Taxi"** - Shoot preload at known angle/RPM, drive away
2. **"Score and Collect"** - Shoot, intake from depot, shoot again
3. **"Score and Climb"** - Shoot, drive to tower, climb

Requires fixing named commands first (current `RunConveyorCommand` blocks auto sequences because it never finishes).

---

## PHASE 2: COMPETITIVE EDGE (~10 hours, after Phase 1 is solid)

### Step 6: Fix and enable vision (~3.5 hours)
Vision-corrected odometry for accurate autos and potential auto-aim.

### Step 7: Beam break integration (~2.5 hours)
Automated fuel detection for smarter feeding (shoot only when loaded, rapid-fire mode).

### Step 8: Distance-based auto-tuning (~2 hours)
Interpolation table from distance → (RPM, hood angle) so the robot auto-adjusts shooting parameters based on position. Uses the characterization data from Phase 0 plus vision for distance.

### Step 9: More autonomous routines (~2 hours)
Multi-ball autos leveraging vision accuracy.

---

## PHASE 3: POLISH (remaining time)

### Step 10: Hub auto-aim (~2 hours)
### Step 11: Shift timer display (~1 hour)
### Step 12: Code cleanup (~0.5 hours)

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

## Suggested Session Schedule

| Session | Hours | Focus | Work Items |
|---------|-------|-------|------------|
| 1 | ~4 hrs | Fix + Characterize | Critical bug fixes + improve test bindings for characterization |
| 2 | ~2 hrs | **ON ROBOT** | Shooting characterization - find RPM + hood values at 3-4 distances |
| 3 | ~4 hrs | Coordinate | Shooting sequence command + intake sequence + competition bindings |
| 4 | ~4 hrs | Automate | Fix named commands + build 3 auto routines + **test on robot** |
| 5 | ~4 hrs | Vision | Fix VisionSubsystem + enable + test with AprilTags |
| 6 | ~4 hrs | Polish | Beam break + distance-based tuning + improved autos |
| Pre-comp | Bring laptop | Tune | Field testing, preset adjustments, path corrections |

---

## Key Risks

| Risk | Mitigation |
|------|-----------|
| Don't find good shooting values | Start with medium distance, work outward. Even 2 good presets is enough for competition. |
| Beam break not wired in time | Shooting sequence works time-based without it |
| Vision estimates noisy | Start with conservative trust levels, tighten after validation |
| Auto paths don't match field | Always have "Score and Taxi" as failsafe auto |
| Intake PID values wrong | Output already clamped to ±0.5 in config, test on blocks first |
| Named command changes break existing autos | Test all autos after changing named command registrations |
