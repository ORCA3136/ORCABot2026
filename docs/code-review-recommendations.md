# ORCABot2026 Code Review — Prioritized Fix List

Comprehensive code review performed 2/20/2026. Identifies bugs, safety issues, and incomplete implementations to address before refining code and autonomous routines.

---

## Priority 1: Show-Stoppers (Robot Won't Function)

### 1b. Auto chooser is null
**File:** `RobotContainer.java:67`
`autoChooser = null;` — autonomous routines cannot be selected or run. `getAutonomousCommand()` on line 146 will NPE.
**Fix:** Uncomment `AutoBuilder.buildAutoChooser()` and set a sensible default.

---

## Priority 2: Bugs (Wrong Behavior)

### 2d. VisionSubsystem angular velocity uses same axis for all 3 components
**File:** `VisionSubsystem.java:162-164, 168-170`
All three angular velocity values use `robotAngularVelocity3dSubscriber.get()[0]` (pitch only). Should be `[0]`, `[1]`, `[2]` for pitch, roll, yaw respectively. This corrupts Limelight orientation data.
**Fix:** Index each axis correctly. Also add bounds checking since the subscriber defaults to an empty array.

### 2e. VisionSubsystem fusion logic is broken — never actually updates odometry
**File:** `VisionSubsystem.java:112, 130-131, 145`
- `bestLimelight` is initialized to `-1` and never reassigned
- When a vision estimate passes the stddev check, `updatePose = true` is set but then `continue` skips past the fitness/bestLimelight logic
- The final check `updatePose && bestLimelight != -1` will therefore never be true
**Fix:** Rewrite the fusion logic so that passing the stddev check actually records which limelight to use.

---

## Priority 3: Missing Functionality

### 3b. DeployIntakeCommand is an empty shell
**File:** `commands/DeployIntakeCommand.java`
`initialize()` and `execute()` are empty — the command does nothing. It's also never bound to a button.
**Fix:** Either implement it (call `intakeSubsystem.deployIntake(true/false)`) or remove it and use inline commands.

### 3c. Missing named commands for autonomous
**File:** `RobotContainer.java:134-135`
No climbing or intake deploy commands are registered. Auto file "Right outpost and climb" explicitly notes it needs these.
**Fix:** Register `"Deploy Intake"`, `"Retract Intake"`, `"Climb"` named commands.

### 3d. Secondary hood motor: follower config vs direct commands conflict
**File:** `ShooterSubsystem.java:142-143`, `Configs.java:34`
The secondary hood motor is configured with `.follow(kHoodPrimaryCanId, true)` in Configs, but `setHoodVelocity()` directly commands both hood motors. This is redundant and could cause unexpected behavior.
Note: The secondary *shooter* motor intentionally does NOT use follow (comment on `Configs.java:24` explains they fight each other when following), so direct commanding of both shooters in `setShooterVelocity()` is correct.
**Fix:** Remove the `hoodSecondaryMotor.set()` call from `setHoodVelocity()` since the follower config handles it.

### 3e. Hood bumper bindings use wrong trigger type
**File:** `RobotContainer.java:110-111`
Right/left bumpers use `whileTrue` with `runOnce`. Since `runOnce` completes instantly, `whileTrue` will rapidly re-fire the command every scheduler cycle. Should be `onTrue` for a one-shot target update.
**Fix:** Change `whileTrue` to `onTrue` for bumper hood target bindings.

---

## Priority 4: Safety Improvements

### 4a. VisionSubsystem lidar will NPE if called
**File:** `VisionSubsystem.java:49, 88`
`lidar` is declared but never initialized (constructor line 70 is commented out). `getLidarMeasurement()` will throw NPE.
**Fix:** Either initialize the lidar, add a null guard, or remove the method if lidar isn't being used.

### 4b. Vision empty array risk
**File:** `VisionSubsystem.java:65, 162-164`
`robotAngularVelocity3dSubscriber` defaults to an empty `double[]`. Accessing `[0]` will throw `ArrayIndexOutOfBoundsException` if no data has been published yet.
**Fix:** Add length check before accessing array elements.

---

## Priority 5: Quality & Cleanup

### 5a. Field positions need verification for 2026 REBUILT
**File:** `Constants.java:205-230`
Field element positions (Hub, Outpost, Tower, Depot, Trenches) appear to be placeholders. `bumpPoses` is empty. These need to match the actual 2026 field dimensions.
**Fix:** Update with official 2026 field coordinates from the game manual field drawings.

### 5b. Inconsistent robot mass between configs
PathPlanner `settings.json` has 61.0 kg, YAGSL `physicalproperties.json` has 110.23 lbs (~50 kg). These should be consistent.
**Fix:** Weigh the robot and update both files to match.

### 5c. VisionSubsystem is fully disabled
**File:** `RobotContainer.java:45, 154-165`
VisionSubsystem is commented out and all LL commands return null. Once Priority 2 vision bugs are fixed, this should be re-enabled.
**Fix:** After fixing 2d and 2e, uncomment VisionSubsystem instantiation and wire up the LL seed/internal commands in Robot.java.

---

## Recommended Order of Work

### Day 1 — Get the robot driving and running autos
- [ ] Fix 1a (uncomment default drive command)
- [ ] Fix 1b (enable auto chooser)
- [ ] Fix 2a, 2b, 2c (copy-paste bugs)
- [ ] Fix 2h, 3a (climber typos + missing NT update)
- [ ] Fix 3e (bumper bindings)

### Day 2 — Fix safety issues and flesh out commands
- [ ] Fix 2g (climber position limits — requires physical measurement)
- [ ] Fix 3b (implement or remove DeployIntakeCommand)
- [ ] Fix 3c (register missing named commands for autos)
- [ ] Fix 3d (resolve hood follower vs direct command conflict)
- [ ] Fix 2f (hood velocity scaling)

### Day 3 — Vision and polish
- [ ] Fix 2d (angular velocity indexing)
- [ ] Fix 2e (rewrite vision fusion logic)
- [ ] Fix 4a, 4b (null guards)
- [ ] Fix 5c (re-enable VisionSubsystem)
- [ ] Fix 5a, 5b (field positions, robot mass)

---

## Verification Checklist

After each batch of fixes:
- [ ] `./gradlew build` — must compile cleanly
- [ ] `./gradlew simulateJava` — verify no NPEs or crashes in simulation
- [ ] Deploy to robot and verify:
  - [ ] Joystick drives the robot (1a)
  - [ ] Auto chooser appears on dashboard (1b)
  - [ ] NetworkTables show correct values for all subsystems
  - [ ] Hood responds to bumper inputs
  - [ ] Climber stops at position limits
