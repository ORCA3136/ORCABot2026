package frc.robot;

import java.util.List;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {



  // Robot max speed
  public static final class RobotConstants {
    public static final double kDriveMaxSpeedMps = 5.33; // Meters per second (Vortex @ 5.08:1 medium pinion)
    public static final double kDriveMaxSpeedFps = 10; // Feet per second

    /** NEO Vortex free speed in RPM. Used to normalize duty cycle values (value / kNeoVortexFreeSpeedRPM). */
    public static final double kNeoVortexFreeSpeedRPM = 6500;
    /** NEO 550 free speed in RPM. Used to normalize duty cycle values for NEO 550 motors. */
    public static final double kNeo550FreeSpeedRPM = 11000;
  }



  public static final class ShooterConstants {
    // Shooter speeds, RPM
    public static final double kVelocityLow = 1900;
    public static final double kVelocityMedium = 3500;
    public static final double kVelocityHigh = 5000;
    public static final double kVelocityMax = 6500;

    public static final double kP = 0.0003;
    public static final double kI = 0.0;
    public static final double kD = 0.01;
    public static final double kS = 0.06;
    public static final double kVelocityModifier = .00181;

    // Setpoint ramp rates (RPM per 20ms cycle)
    // At 200 RPM/cycle spin-up: 0 → 5000 RPM takes ~0.5 seconds
    // At 400 RPM/cycle spin-down: 5000 → 0 RPM takes ~0.25 seconds
    public static final double kRampUpRate = 200.0;
    public static final double kRampDownRate = 400.0;

    // Flywheel is "ready" when within this tolerance of target RPM
    public static final double kReadyToleranceRPM = 200;

    // Lead compensation gain — multiplier for heading offset magnitude (tune on field)
    public static final double kLeadCompGain = 1.0;
  }

  public static final class IntakeConstants {
    // Intake roller speeds, RPM
    public static final double kVelocityLow = 500;
    public static final double kVelocityMedium = 1000;
    public static final double kVelocityHigh = 1500;
    public static final double kVelocityMax = 2500;

    // Rack & pinion deploy PID (on SparkFlex relative encoder, position control)
    public static final double kP = 0.2;
    public static final double kI = 0.0;
    public static final double kD = 0.025;

    // Gravity feedforward for extending (mechanism angles slightly downward)
    public static final double kExtendArbFF = 0.05;     // duty cycle offset for extending against gravity
    public static final double kArbFFDeadband = 0.05;    // rotations — prevents toggling at setpoint

    // Rack & pinion gear ratio: 9:1 gearbox * 18T→22T sprocket * 12T lantern→32T rack
    public static final double kRackGearRatio = 9.0 * (22.0 / 18.0); // ≈11.0:1
    public static final double kIntakeGearRatio = 1;

    // Full travel in motor rotations: (32T/12T) × 9:1 gearbox = 24.0
    public static final double kFullTravelMotorRotations = (32.0 / 12.0) * 9.0; // 24.0

    // Linear positions in motor rotations from home (0 = fully retracted)
    public static final double kRetractedPosition = -0.25;
    public static final double kShuttleCenter = 12.5;      // Center point for shuttle pulse (halfway)
    public static final double kExtendedPosition = 25.0;
    public static final double kMaxExtension = 26.0;

    public static final double kTargetTolerance = 0.25;

    // Deploy position ramp rates (motor rotations per 20ms cycle)
    // Bumped for longer travel distance
    public static final double kExtendRampRate = 1.0;
    public static final double kRetractRampRate = 1.1;

    // Slow retract for feeding: motor rotations per 20ms cycle
    // Gradually pulls intake in to feed fuel toward conveyor
    public static final double kFeedRetractRate = 0.084;

    // Homing
    public static final double kHomingDutyCycle = -0.1;    // Slow inward (negative = retract)
    public static final double kHomingCurrentThreshold = 20.0; // Amps — stall detection
    public static final double kStallCurrentThreshold  = 38.0; // Amps — stall detection (above normal PID servo current)
    public static final int kStallCycles = 100;            // Consecutive cycles above threshold (~2000ms)
    public static final int kHomingStallCycles = 13;        // Consecutive cycles above threshold (~260ms)
    public static final double kHomingTimeoutSec = 5.0;    // Max time before FAULT
    public static final double kPIDTimeoutSec = 0.1;    // Max time before FAULT

    // Stall recovery — reverse and retry instead of immediate FAULT
    public static final int kMaxStallRetries = 3;
    public static final double kStallReverseRotations = 1.5;      // Motor rotations to reverse
    public static final double kStallRecoveryTimeoutSec = 1.0;    // Max time for recovery move

    // Set to true when the limit switch is physically wired on the robot
    public static final boolean kLimitSwitchInstalled = true;

    // Shuttle pulse for feeding — in/out motion between shuttle center and near-retracted
    public static final double kPulseAmplitude = 5.4;      // Motor rotations peak-to-peak (shuttle travel)
    public static final double kPulseFrequencyHz = 1.0;    // Slow deliberate in/out motion

    // PID output clamp to prevent slamming into hard stops
    public static final double kMaxOutputDutyCycle = 0.8;

    // ── Fuel detection (roller current spike) ───────────────────────
    // Threshold must be above normal free-running current (~5-8A) but below
    // the deploy stall threshold (38A). Tune with URCL current data.
    public static final double kFuelDetectCurrentAmps    = 17.0; // Amps — spike threshold
    public static final int    kFuelDetectDebounceCycles = 2;    // Consecutive cycles above threshold (~40ms at 50Hz)
  }

  public static final class VisionConstants {
    // Camera names — must match the names configured in the Limelight web interface.
    public static final String kLimelightFrontName = "limelight-front";
    public static final String kLimelightBackName = "limelight-back";

    // --- Rejection thresholds ---
    public static final double kMaxYawRateDegPerSec = 270.0;
    public static final double kMaxTagDistanceM = 6.0;
    public static final double kSingleTagMaxDistanceM = 4.0;
    public static final double kMaxHeadingErrorDeg = 60.0;
    public static final double kMaxTimestampAgeSec = 0.5;

    // 2026 REBUILT field boundary + 0.5m margin (field is 16.54m x 8.21m)
    public static final double kFieldMinX = -0.5;
    public static final double kFieldMaxX = 17.04;
    public static final double kFieldMinY = -0.5;
    public static final double kFieldMaxY = 8.71;

    // --- Std dev tuning ---
    public static final double kXYStdDevBase = 0.5;
    public static final double kSingleTagPenalty = 2.0;
    public static final double kMinXYStdDev = 0.1;

    // --- IMU settling ---
    public static final int kImuSettleCycles = 15; // ~300ms at 50Hz

    public static final double kVisionHealthyTimeoutSec = 0.2;
    public static final double kTagDistanceStaleSec = 0.08;

    // Minimum conditions for disabled-period odometry seeding
    public static final int kSeedMinTagCount = 2;
    public static final double kSeedMaxDistM = 4.0;

    // --- Dual-camera drift detection & trust boost ---
    public static final double kDriftDetectionThresholdM = 0.75;
    public static final double kCameraAgreementMaxM = 0.2;
    public static final int kDriftConfirmCycles = 10;     // ~200ms at 50Hz
    public static final double kBoostedXYStdDev = 0.02;
    public static final int kBoostDurationCycles = 3;    // ~60ms at 50Hz
    public static final double kHardResetThresholdM = 1.5;
    public static final int kHardResetConfirmCycles = 10; // ~200ms at 50Hz
    public static final double kDriftPoseFreshnessMaxSec = 0.2;

    // --- Dual-camera same-cycle agreement ---
    public static final double kDualCameraAgreementThresholdM = 0.25;
    public static final double kDualCameraAgreedStdDev = 0.03;

    // --- Single-camera-unavailable trust ---
    public static final int kSingleCameraBoostMinTags = 2;
    public static final double kSingleCameraBoostMaxDistM = 3.0;
    public static final double kSingleCameraBoostStdDev = 0.06;
    public static final int kSingleCameraHardResetMinTags = 3;
    public static final double kSingleCameraHardResetMaxDistM = 2.5;
    public static final int kSingleCameraHardResetCycles = 20; // ~400ms at 50Hz

    // --- MT1 heading recovery ---
    public static final int kMT1RecoveryCycles = 50;        // ~1s of sustained MT1 flip before auto-recovery
    public static final int kMT1FastRecoveryCycles = 3;    // Fast recovery when pigeon CAN is unhealthy (~60ms)
    public static final double kPitchGateThresholdDeg = 10.0; // suppress recovery while pitch > this

    // --- Camera health monitoring ---
    public static final double kCameraStaleThresholdSec = 5.0;
  }
  
  public static final class OperatorConstants {
    // Controller and button constants

    // Drive controller port numbers
    public static final int kDriverControler = 0;
    public static final int kSecondaryDriverControler = 1;

    public static final double kStickDeadband = 0.04;
    public static final double kPathplannerDeadband = 0.01;
    public static final double kTriggerDeadband = 0.05;

    // ── Driver rumble feedback ──────────────────────────────────────
    // Rumble provides tactile feedback through the Xbox controller.
    // Intensity ranges from 0.0 (off) to 1.0 (max vibration).

    // Fuel detection — single pulse when intake roller picks up fuel
    public static final double kFuelRumbleIntensity   = 0.5; // Rumble strength (0.0 – 1.0)
    public static final double kFuelRumbleDurationSec = 0.5; // Duration of each pulse

    // Deploy stall warning — repeating pulse while deploy motor is near stall
    // Pattern: ON for kStallRumblePulseSec → OFF for kStallRumblePauseSec → repeat
    // Tied to IntakeConstants.kStallCurrentThreshold (35A)
    public static final double kStallRumbleIntensity  = 1.0; // Rumble strength (0.0 – 1.0)
    public static final double kStallRumblePulseSec   = 1.0; // Rumble ON duration per pulse
    public static final double kStallRumblePauseSec   = 0.3; // Gap between pulses
  }

  public static final class CanIdConstants {

    // Drive Vortexes
    public static final int kFrontRightDrivingCanId = 1;
    public static final int kFrontLeftDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 5;
    public static final int kRearLeftDrivingCanId = 7;

    // Drive Turning 550s
    public static final int kFrontRightTurningCanId = 2;
    public static final int kFrontLeftTurningCanId = 4;
    public static final int kRearRightTurningCanId = 6;
    public static final int kRearLeftTurningCanId = 8;

    // Pigeon
    public static final int kPigeonCanId = 9;

    // Shooter Vortexes
    public static final int kShooterPrimaryCanId = 11;
    public static final int kShooterSecondaryCanId = 12;

    // Kicker Vortex
    public static final int kKickerCanId = 13;

    // Conveyor Vortex
    public static final int kConveyorCanId = 14;

    // Intake Vortex
    public static final int kIntakeCanId = 17;

    // Intake Deployment 550
    public static final int kIntakeDeployCanId = 18;

    // PDH
    public static final int kPDHCanId = 50;
  }

  public static final class DioConstants {
    public static final int kIntakeHomeLimitSwitchPort = 0;
    public static final int kBeamBreakPort = 1;
  }

  public static final class NetworkTableNames {

    public static final class Robot {
      public static final String kTable = "Robot";
      public static final String kEnabled = "Enabled";
      public static final String kBatteryVoltage = "Battery Voltage";
      public static final String kBatteryBrownout = "Battery Brownout Setting";
    }
      
    public static final class Odometry {
      public static final String kTable = "Odometry";
      public static final String kRobotPose2d = "Robot Pose 2d";
      public static final String kRobotRotation3d = "Robot Pitch, Roll, Yaw Rotations";
      public static final String kRobotAngularVelocity3d = "Robot Pitch, Roll, Yaw Velocities";
      public static final String kRobotVelocity = "Robot X, Y, Z Velocities";
      public static final String kDistanceToHubMeters = "Distance to Hub meters";
      public static final String kDistanceToHubInches = "Distance to Hub Inches";
      public static final String kDistanceToTrench = "Distance to Nearest Trench";
      public static final String kNavxHeading = "Navx heading";
    }

    public static final class Vision {
      public static final String kTable = "Vision";

      // Per-camera table prefixes
      public static final String kFrontTable = "Vision/Front";
      public static final String kBackTable = "Vision/Back";

      // Per-camera keys
      public static final String kVisionPose = "VisionPose";
      public static final String kTagCount = "TagCount";
      public static final String kAvgTagDistance = "AvgTagDistance";
      public static final String kAccepted = "Accepted";
      public static final String kRejectReason = "RejectReason";

      // Per-camera MT1 pose key
      public static final String kVisionPoseMT1 = "VisionPoseMT1";

      // System-level keys
      public static final String kVisionHealthy = "VisionHealthy";
      public static final String kImuSettled = "ImuSettled";
      public static final String kOdometryHeading = "OdometryHeading";
      // Removed: PigeonRawYaw, DetectedAlliance, FirstFixStatus, ImuMode, AprilTagReady
      // — logged as string events on state change instead of per-cycle NT entries
      public static final String kAllianceOverride = "AllianceOverride";
      public static final String kTotalTagCount = "TotalTagCount";

      // Heading cross-check
      public static final String kHeadingError = "HeadingErrorDeg";

      // Drift detection telemetry
      public static final String kDriftDetected = "DriftDetected";
      public static final String kDriftMagnitude = "DriftMagnitudeM";
      public static final String kCameraAgreement = "CameraAgreement";
      public static final String kTrustBoosted = "TrustBoosted";
      public static final String kHardResetCount = "HardResetCount";

      // Dual-camera agreement
      public static final String kDualCameraAgreed = "DualCameraAgreed";

      // --- Enhanced logging for AdvantageScope ---

      // Per-camera: fused std dev, tag IDs
      public static final String kFusedStdDevXY = "FusedStdDevXY";
      public static final String kTagIDs = "TagIDs";

      // System-level: fusion mode, pre-fusion pose, vision delta
      public static final String kFusionMode = "FusionMode";
      public static final String kPreFusionPose = "PreFusionPose";
      public static final String kVisionDeltaM = "VisionDeltaM";

      // Rejection counters
      public static final String kCountsTable = "Vision/Counts";
      public static final String kCountAccepted = "Accepted";
      public static final String kCountStale = "StaleTimestamp";
      public static final String kCountOutOfField = "OutOfField";
      public static final String kCountYawRate = "HighYawRate";
      public static final String kCountTooFar = "TooFar";
      public static final String kCountSingleTagFar = "SingleTagFar";
      public static final String kCountHeadingFlip = "HeadingFlip";
      public static final String kCountNoData = "NoData";
    }

    public static final class Shooter {
      public static final String kTable = "Shooter";
      public static final String kVelocityRPM = "Velocity RPM";
      public static final String kTargetRPM = "Target RPM";
      public static final String kRampedSetpoint = "Ramped Setpoint RPM";
      public static final String kPrimaryCurrent = "Primary Current";
      public static final String kSecondaryCurrent = "Secondary Current";
      public static final String kLeadCompX = "Lead Comp X";
      public static final String kLeadCompY = "Lead Comp Y";
      public static final String kLeadCompDistance = "Lead Comp Distance";
      public static final String kAirTimeSec = "Air Time Sec";
      public static final String kActualDistanceM = "Actual Distance M";
      public static final String kFieldVelX = "Field Vel X";
      public static final String kFieldVelY = "Field Vel Y";
      public static final String kLeadCompGain = "Lead Comp Gain";
    }

    public static final class Conveyor {
      public static final String kTable = "Conveyor";
      public static final String kVelocityRPM = "Velocity RPM";
      public static final String kCurrentAmps = "Current Amps";
    }

    public static final class Kicker {
      public static final String kTable = "Kicker";
      public static final String kVelocityRPM = "Velocity RPM";
      public static final String kCurrentAmps = "Current Amps";
      public static final String kStallStatus = "Stall Status";
      public static final String kFuelStaged = "Fuel Staged";
    }

    public static final class Intake {
      public static final String kTable = "Intake";
      public static final String kVelocityRPM = "Velocity RPM";
      public static final String kCurrentAmps = "Current Amps";
      
    }

    public static final class IntakeDeploy {
      public static final String kTable = "IntakeDeploy";
      public static final String kVelocityRPM = "Velocity RPM";
      public static final String kPositionRotations = "Position Rotations";
      public static final String kVoltageRotations = "Voltage";
      public static final String kCurrentAmps = "Current Amps";
      public static final String kTarget = "Intake Deploy Target";
      public static final String kRampedPosition = "Ramped Position";
      public static final String kState = "State";
      public static final String kFaultReason = "Fault Reason";
      public static final String kLimitSwitch = "Limit Switch";
      public static final String kAbsEncoderRaw = "Abs Encoder Raw";
    }

  }

  public static final class FieldPositions {

    public static final List<Translation2d> kBlueFieldElements = List.of(
      new Translation2d(4.597,  4.035),         // Blue Hub
      new Translation2d(0.2,    0.666),         // Blue Outpost
      new Translation2d(1.065,  3.745),         // Blue Tower
      new Translation2d(0.410,  5.965)          // Blue Depot
    );

    public static final List<Translation2d> kRedFieldElements = List.of(
      new Translation2d(11.938, 4.035),         // Red Hub
      new Translation2d(16.621, 7.403),         // Red Outpost
      new Translation2d(15.48,  3.745),         // Red Tower
      new Translation2d(16.13,  2.103)          // Red Depot
    );

    public static final List<Translation2d> kTrenchPoses = List.of(
      new Translation2d(4.620, 7.430),
      new Translation2d(4.620, 0.630),
      new Translation2d(11.93, 7.430),
      new Translation2d(11.93, 0.630)
    );

    public static final Pose2d kTestPosition = new Pose2d(13.9, 4.1, Rotation2d.fromDegrees(180));

    // TODO: Populate with actual bump field coordinates
    public static final List<Translation2d> kBumpPoses = List.of();

    public static final List<Translation2d> kBlueShootingPoses = List.of(
      new Translation2d(4.120, 7.430),    // Top trench shot
      new Translation2d(4.120, 0.630)     // Bottom trench shot
    );

    public static final List<Translation2d> kRedShootingPoses = List.of(
      new Translation2d(12.53, 7.430),    // Top trench shot
      new Translation2d(12.43, 0.630)     // Bottom trench shot
    );

    // --- Drive-to-position targets (blue-origin; alliance flipping handles red side) ---
    // Offsets and headings are initial estimates — tune on the field.

    /** Shooting position ~1.5m from hub, facing toward hub */
    public static final Pose2d kHubPose = new Pose2d(3.1, 4.035, Rotation2d.fromDegrees(0));

    /** Climbing approach ~0.5m from tower base, facing tower */
    public static final Pose2d kTowerPose = new Pose2d(1.6, 3.745, Rotation2d.fromDegrees(0));

    /** Human player pickup near outpost, facing outpost */
    public static final Pose2d kOutpostPose = new Pose2d(0.7, 0.666, Rotation2d.fromDegrees(-45));

    /** Floor fuel pickup near depot, facing depot */
    public static final Pose2d kDepotPose = new Pose2d(0.9, 5.965, Rotation2d.fromDegrees(45));

    /** Left trench entrance, heading along trench */
    public static final Pose2d kLeftTrenchPose = new Pose2d(4.620, 7.430, Rotation2d.fromDegrees(90));

    /** Right trench entrance, heading along trench */
    public static final Pose2d kRightTrenchPose = new Pose2d(4.620, 0.630, Rotation2d.fromDegrees(-90));

    // ── Scoring positions (blue-origin, PathPlanner flips for Red) ──────
    // Left/Right are from the DRIVER STATION perspective (Left = high Y, Right = low Y)
    // Heading faces the shooter toward the hub

    // Close: ~1.5m from hub, high percentage shot
    public static final Pose2d kScoreCloseLeft = new Pose2d(3.5, 5.5, Rotation2d.fromDegrees(30));   // TODO: MEASURE ON FIELD
    public static final Pose2d kScoreCloseRight = new Pose2d(3.5, 2.5, Rotation2d.fromDegrees(-30)); // TODO: MEASURE ON FIELD

    // Trench: shot from trench zone, medium range
    public static final Pose2d kScoreTrenchLeft = new Pose2d(4.6, 7.0, Rotation2d.fromDegrees(60));  // TODO: MEASURE ON FIELD
    public static final Pose2d kScoreTrenchRight = new Pose2d(4.6, 1.0, Rotation2d.fromDegrees(-60));// TODO: MEASURE ON FIELD

    // Far: long range fallback, ~4m from hub
    public static final Pose2d kScoreFarLeft = new Pose2d(2.0, 6.0, Rotation2d.fromDegrees(45));     // TODO: MEASURE ON FIELD
    public static final Pose2d kScoreFarRight = new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(-45));   // TODO: MEASURE ON FIELD

    // Custom: tested scoring position (blue-side), heading faces hub
    public static final Pose2d kCustomScoringPose = new Pose2d(2.153, 1.954, Rotation2d.fromDegrees(40.4));

    // Driver override threshold — higher than normal deadband (0.08)
    // so minor stick wobble doesn't accidentally cancel auto-drive
    public static final double kDriverOverrideDeadband = 0.15;

    // Scoring command timeout
    public static final double kScoringDriveTimeoutSec = 5.0;
  }

   public final static class CurrentConstants {
  
    public static final int AMP80 = 80;
    public static final int AMP60 = 60;
    public static final int AMP40 = 40;
    public static final int AMP30 = 30;
    public static final int AMP25 = 25;
    public static final int AMP20 = 20;
    public static final int AMP15 = 15;
    public static final int AMP10 = 10;

  }

  public static final class AutoConstants {
    public static final double kIntakeTimeoutSec = 3.0;
    public static final double kShootTimeoutSec = 5.0;
    public static final double kFeedTimeoutSec = 3.0;
    public static final double kAimTimeoutSec = 1.5;
  }

  public static final class PathPlannerConstants {
    public static final PathConstraints slowConstraints = new PathConstraints(
        Units.feetToMeters(3.5), 4.0,             
        Units.degreesToRadians(100), Units.degreesToRadians(720));
  }

  public static final class FuelPathConstants {
    // Intake roller speeds (RPM)
    public static final double kIntakeInSlow = 2000;
    public static final double kIntakeInStandard = 5000;
    public static final double kIntakeInFast = 6000;
    public static final double kIntakeOutSlow = -1500;
    public static final double kIntakeOutStandard = -3000;
    public static final double kIntakeOutFast = -5000;

    // Conveyor speeds (RPM)
    public static final double kConveyorIn = 2000;
    public static final double kConveyorOut = -1000;

    // Conveyor jog (nudge to clear minor blockages)
    public static final double kConveyorJogSpeed = 400;
    public static final double kConveyorJogDurationSec = 0.15;

    // Kicker speeds (RPM)
    public static final double kKickerFeed = 4000;
    public static final double kKickerOut = -2000;

    // Kicker pulse (single-ball burst)
    public static final double kKickerPulseSpeed = 3000;
    public static final double kKickerPulseDurationSec = 0.15;

    // Jam detection
    // TODO: TUNE ON ROBOT — start at 55A, tune down with URCL current data from practice matches
    public static final double kJamCurrentThresholdAmps = 55.0;
    public static final double kJamDetectionTimeSec = 0.25;
    public static final double kJamReverseDurationSec = 0.3;
    public static final double kJamReverseSpeed = -2000;
    public static final int kJamMaxRetries = 3;

    // Intake pulse (free jammed fuel)
    public static final double kIntakePulseSpeed = 3000;
    public static final double kIntakePulseDurationSec = 0.5;
    public static final double kIntakePulsePauseSec = 0.5;

    // Emergency reverse (all motors reverse at high speed)
    public static final double kEmergencyIntakeSpeed = -5000;
    public static final double kEmergencyConveyorSpeed = -2000;
    public static final double kEmergencyKickerSpeed = -3000;
  }

  public static final class LedColors {

    //These are all the led options, if you want more you will have to go to a REV website called "LED BLINKIN DRIVER"

    //Patterns:
    public static final double Rainbow_Rainbow_Pallet = -0.99;
    public static final double Rainbow_Ocean_Pallet = -0.95;
    public static final double Rainbow_Forest_Pallet = -0.91;
    public static final double Fire_Medium = -0.59;
    public static final double Fire_Large = -0.57;

    //Basic colors:
    public static final double Hot_Pink = 0.57;
    public static final double Dark_Red = 0.59;
    public static final double Red = 0.61;
    public static final double Red_Orange = 0.63;
    public static final double Orange = 0.65;
    public static final double Gold = 0.67;
    public static final double Yellow = 0.69;
    public static final double Lawn_Green = 0.71;
    public static final double Lime = 0.73;
    public static final double Dark_Green = 0.75;
    public static final double Green = 0.77;
    public static final double Blue_Green = 0.79;
    public static final double Aqua = 0.81;
    public static final double Sky_Blue = 0.83;
    public static final double Dark_Blue = 0.85;
    public static final double Blue = 0.87;
    public static final double Blue_Violet = 0.89;
    public static final double Violet = 0.91;
    public static final double White = 0.93;
    public static final double Gray = 0.95;
    public static final double Dark_Gray = 0.97;
    public static final double Black = 0.99;
  }
}
