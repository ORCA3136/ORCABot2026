# Code Review TODO

Generated 2026-02-25 from full codebase review. Items are ordered by priority.

## CRITICAL — Fix Before Competition

### Configs.java: Shooter PID uses wrong feedback sensor
- **File:** `Configs.java:26-31`
- **Issue:** `kDetachedRelativeEncoder` tells SparkFlex to read from nonexistent external encoder. PID reads zero, output saturates — shooter runs full power uncontrolled.
- **Fix:** Change to `FeedbackSensor.kPrimaryEncoder` for both shooter configs.

### Configs.java: Hood position wrapping enabled
- **File:** `Configs.java:53`
- **Issue:** Hood is limited-travel, not continuous rotation. Wrapping can command the mechanism through its hard stop.
- **Fix:** Change `positionWrappingEnabled(true)` to `false`.

### Configs.java: Intake deploy position wrapping enabled
- **File:** `Configs.java:99`
- **Issue:** Same as hood — intake deploy is bounded 0.0 to 0.5.
- **Fix:** Change `positionWrappingEnabled(true)` to `false`.

### Configs.java: Hood motors in Coast mode
- **File:** `Configs.java:42-46`
- **Issue:** Hood is gravity-loaded. Coast mode = freefall when disabled, slamming into hard stops.
- **Fix:** Change `IdleMode.kCoast` to `IdleMode.kBrake` for both hood configs.

### HoodSubsystem: Secondary motor commanded while configured as follower
- **File:** `HoodSubsystem.java:101-103`
- **Issue:** `setHoodVelocity()` calls `.set()` on both motors, but secondary is a follower. Motors fight each other.
- **Fix:** Remove `hoodSecondaryMotor.set(velocity / 11000)` — follower handles it automatically.

### Hood encoder/target units mismatch
- **File:** `Configs.java:49-51`, `HoodSubsystem.java:70-71,117`
- **Issue:** Encoder position scale (12x conversion factor) doesn't match target calculation scale (26.67x). PID setpoint and feedback are in different unit spaces.
- **Fix:** Needs investigation on physical robot — determine where absolute encoder is mounted and make both sides consistent.

### RobotContainer: Auto chooser never published to dashboard
- **File:** `RobotContainer.java:76`
- **Issue:** `autoChooser` built but never sent to SmartDashboard. Drive team can't select autos.
- **Fix:** Add `SmartDashboard.putData("Auto Chooser", autoChooser);` after line 76. Add `import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;`.

---

## WARNING — Fix Before Competition

### RobotContainer: Only test bindings active
- **File:** `RobotContainer.java:73`
- **Issue:** `configureTestBindings()` called instead of `configureBindings()`. Test mode is missing intake deploy, climber, proper operator controls.
- **Fix:** Switch to `configureBindings()` for competition, or add a toggle.

### RobotContainer: Operator button board entirely commented out
- **File:** `RobotContainer.java:66`
- **Issue:** Secondary controller has zero bindings. Operator can't control mechanisms.
- **Fix:** Uncomment, fix reference to `Constants.OperatorConstants.kSecondaryDriverControler`, and add operator bindings.

### Constants: bumpPoses list is empty
- **File:** `Constants.java:311-312`
- **Issue:** `getDistanceToNearestBump()` will throw `NoSuchElementException` on empty list.
- **Fix:** Either populate with actual bump coordinates or add empty-list guard in `SwerveSubsystem.getDistanceToNearestBump()`.

### SwerveSubsystem: getAlliance() returns null
- **File:** `SwerveSubsystem.java:299-303`
- **Issue:** Returns null when alliance not yet set. Callers of `getTranslationToFieldElement()` risk NPE.
- **Fix:** Return `Optional<Alliance>` or add null guard in callers.

### Shooter feedforward kS=1.0 too high
- **File:** `Constants.java:35`, `ShooterSubsystem.java:62-66`
- **Issue:** At 5000 RPM: `ff = 1.0 + 5000 * 0.002 = 11.0` — 11V on 12V system. PID has no headroom. Effectively open-loop.
- **Fix:** Typical kS ~0.05-0.2, kV ~0.00015. Needs characterization (SysId or manual).

### Climber: No software position limits
- **File:** `Constants.java:83-85`, `RunClimberCommand.java:54-61`
- **Issue:** Position limits commented out and both set to 0. Overextending can break mechanism.
- **Fix:** Measure real positions, set constants, uncomment `isFinished()` checks. Consider SparkFlex soft limits.

### IntakeSubsystem: Deploy PID commented out
- **File:** `IntakeSubsystem.java:227`
- **Issue:** `setPIDAngle()` commented out. `deployIntake()` and named commands are silently no-ops.
- **Fix:** Uncomment when PID is tuned, or change named commands to use `setIntakeDeployVelocity()` as workaround.

### Named commands: "Stop Intake" and "Stop Conveyor" hold subsystems indefinitely
- **File:** `RobotContainer.java:171,192`
- **Issue:** Uses `RunIntakeCommand(0)` / `RunConveyorAndKickerCommand(0,0)` which never finish. Holds subsystem, blocking future commands.
- **Fix:** Change to `Commands.runOnce(() -> subsystem.setVelocity(0), subsystem)`.

### SlowHoodMove: Units confusion and dead code
- **File:** `SlowHoodMove.java`
- **Issue:** `slowHoodSpeed` documented as "degrees/sec" but added to a value in rotations. `getClass()` call in `initialize()` is dead code. Magic numbers 5 and 1 for boundary check.
- **Fix:** Fix units, extract constants, remove dead code.

### Open-loop set(velocity / 6500) pattern is misleading
- **Files:** `ConveyorSubsystem.java:55`, `KickerSubsystem.java:60`, `ClimberSubsystem.java:58`, `IntakeSubsystem.java:168,182`, `HoodSubsystem.java:101`
- **Issue:** Methods named `setXxxVelocity(RPM)` but actually set duty cycle. Speed varies with battery voltage.
- **Fix (short-term):** Rename to `setXxxPower()` or document the behavior. **(long-term):** Add closed-loop velocity PID.

### TeleopPathplanner: createTrenchPathCommand() returns null
- **File:** `TeleopPathplanner.java:77-89`
- **Issue:** `Commands.defer()` returns null command — scheduler will crash if caller is uncommented.
- **Fix:** Return `Commands.none()` instead of null, or delete the stub method.

### Jam current threshold may be too low
- **File:** `Constants.java:354`
- **Issue:** 35A threshold may false-trigger under normal multi-ball load (NEO Vortex normal loaded = 20-50A).
- **Fix:** Start at 55A and tune down with URCL current data from practice matches.

---

## SUGGESTIONS — Nice-to-Have

### Cache NetworkTable entries as fields
- **Files:** All subsystems except VisionSubsystem
- **Issue:** `getEntry()` creates wrapper objects every cycle (~1750/sec across all subsystems).
- **Fix:** Declare `private final NetworkTableEntry` fields, assign in constructor. VisionSubsystem already does this correctly.

### Delete dead files
- `ExampleCommand.java`, `ExampleSubsystem.java` — WPILib templates, serve no purpose
- `ShooterSelfFeedCommand.java` — completely empty, holds subsystem requirements for nothing

### Use List.of() instead of double-brace initialization
- **File:** `Constants.java:290-312`
- **Issue:** Double-brace init creates anonymous inner classes. `List.of()` is cleaner and immutable.

### Clean up unused imports
- `ShooterSubsystem.java` — AbsoluteEncoder, SparkMax, Units, Velocity
- `HoodSubsystem.java` — RelativeEncoder, SparkFlex, Units, Velocity
- `IntakeSubsystem.java` — massive block of commented-out simulation imports
- `Constants.java` — RobotController
- `ClimberSubsystem.java` — AbsoluteEncoder, SparkClosedLoopController
- `RobotContainer.java` — Subsystem, CommandJoystick (used only in commented code)

### Add .withName() to individual FuelPathCommands wrappers
- **File:** `FuelPathCommands.java:25-67`
- **Issue:** `intakeIn()`, `kickerFeed()`, etc. show as generic `RunKickerCommand` in command logs. Can't distinguish speed tiers.

### LedSubsystem never instantiated
- **File:** `LedSubsystem.java`
- **Issue:** Full implementation but never created in RobotContainer. Either instantiate or delete.
