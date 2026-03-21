# Shoot-on-the-Fly Implementation Plan

## Context
The team wants to shoot FUEL while the robot is moving — keeping the shooter aimed at the hub center and dynamically adjusting flywheel RPM to compensate for the robot's velocity. The current system (`setShooterMapOnly()`) only works well for stationary shots because it doesn't account for the ball inheriting the robot's velocity during flight.

## Core Concept: Virtual Target Point

Instead of aiming at the real hub, compute a **virtual target point** that compensates for robot motion during ball flight:

```
virtualTarget = hubPosition - robotVelocity * flightTime
```

This single formula handles both:
- **Heading offset** (tangential compensation) — aim ahead of the hub
- **RPM adjustment** (radial compensation) — use distance-to-virtual-target for RPM lookup

The ball inherits the robot's velocity, curving back to hit the real hub.

## Data Already Available
- Robot field velocity: `SwerveSubsystem.getFieldVelocity()` (ChassisSpeeds)
- Robot pose: `SwerveSubsystem.getPose()`
- Distance to hub: `SwerveSubsystem.getDistanceToHubMeters()`
- RPM interpolation table: `ShooterSubsystem.shooterSpeedOnlyMap` (distance → RPM)
- Hub field coordinates: `Constants.FieldPositions` (Blue: 4.597, 4.035 / Red: 11.938, 4.035)

## Data You Must Measure on the Robot

### 1. Ball Flight Time (REQUIRED — highest priority)
Measure time from ball leaving shooter to entering hub at 4-5 distances:

| Distance | Expected Flight Time |
|----------|---------------------|
| 60"      | ~0.30s              |
| 100"     | ~0.45s              |
| 140"     | ~0.58s              |
| 180"     | ~0.70s              |
| 200"     | ~0.78s              |

**How to measure:** Use slow-motion video (240fps phone camera) at each distance. Record the frame the ball leaves the shooter and the frame it enters the hub. Count frames / framerate = flight time.

These populate an `InterpolatingDoubleTreeMap` (same structure as the RPM table). This is the single most critical input — if flight times are wrong by 30%+, compensation will aim at the wrong point.

### 2. No Other Data Needed
The hood is fixed angle, so there's no hood adjustment to worry about.
Everything else (velocity vectors, pose, heading, distance calculations) is already available in the codebase.

---

## Implementation Steps

### Step 1: Add Constants
**File:** `Constants.java` — new inner class `ShootOnTheFlyConstants`
- `kMaxCompensationSpeedMps = 2.5` — above this, model is unreliable
- `kMinCompensationSpeedMps = 0.3` — below this, use static shooting (prevents jitter)
- `kMaxHeadingLeadRad = Math.toRadians(25)` — safety clamp on aim offset
- `kMaxVirtualDistanceOffsetM = 1.5` — safety clamp on distance adjustment

### Step 2: Add Flight Time Map + Methods to ShooterSubsystem
**File:** `src/main/java/frc/robot/subsystems/ShooterSubsystem.java`
- Add `flightTimeMap` (`InterpolatingDoubleTreeMap`) with placeholder values (TODO: replace with measured)
- Add `getFlightTimeSeconds(double distanceMeters)` — public getter for command use
- Add `setShooterVirtualDistance(double virtualDistanceMeters)` — sets RPM from virtual distance, clamped to table range
- Add NT entries for virtual distance, flight time, compensation active flag

### Step 3: Add Virtual Target Helpers to SwerveSubsystem
**File:** `src/main/java/frc/robot/subsystems/SwerveSubsystem.java`
- `getHubTranslation()` — returns alliance-aware hub Translation2d (DRY refactor of existing logic)
- `getVirtualTarget(double flightTimeSec)` — returns `hubPos - fieldVelocity * flightTime`
- `getHeadingToTarget(Translation2d target)` — heading angle from robot to any target point
- `getDistanceToTarget(Translation2d target)` — distance from robot to any target point

### Step 4: Create ShootOnTheFlyCommand
**File (new):** `src/main/java/frc/robot/commands/ShootOnTheFlyCommand.java`

Each cycle (20ms):
1. Get actual distance to hub
2. Look up flight time from that distance
3. Compute virtual target = hub - velocity * flightTime
4. Compute virtual distance to virtual target
5. Set shooter RPM from virtual distance
6. Compute and store aim heading toward virtual target
7. Falls back to static `setShooterMapOnly()` when robot speed < 0.3 m/s or > 2.5 m/s

Exposes `getAimHeading()` for the drive system to use.

### Step 5: Wire into RobotContainer
**File:** `src/main/java/frc/robot/RobotContainer.java`
- Bind `ShootOnTheFlyCommand` to a trigger (e.g., right trigger replaces current `ShootCommand`)
- Pair with YAGSL heading control that aims at the virtual target heading
- Keep existing `ShootCommand`/`ShootOnlyCommand` as fallbacks on different buttons
- Register `"Shoot On The Fly"` as a named command for PathPlanner

### Step 6: Add Telemetry
- Publish virtual distance, flight time, heading lead, compensation active to NT
- Essential for tuning in AdvantageScope

## Verification
1. **Simulation:** Deploy in sim, verify virtual target math produces reasonable values (virtual target shifts opposite to robot motion)
2. **Static baseline:** Confirm static shooting (robot stationary) is unchanged — command falls back to `setShooterMapOnly()`
3. **Field testing:** Drive at known speeds past the hub, observe if shots land closer to center than without compensation
4. **Tune:** Adjust flight time values based on observed miss patterns. If shots consistently overshoot left/right, flight times need adjustment.

## Potential Challenges
- **Shooter ramp rate (200 RPM/cycle)** may lag behind rapidly changing virtual distance — consider increasing ramp rate for moving shots
- **YAGSL heading PID** must be responsive enough to track changing aim heading — test existing tuning first
- **Flight time accuracy is critical** — start conservative, refine with video analysis
