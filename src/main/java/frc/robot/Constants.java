package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {



  // Robot max speed
  public static final class RobotConstants {
    public static final double kDriveMaxSpeedMps = 3.05; // Meters per second
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

    public static final double kP = 0.001;
    public static final double kI = 0.;
    public static final double kD = 0.25;
    public static final double kG = 0.;
    // TODO: TUNE ON ROBOT — characterize with SysId. Typical kS = 0.05–0.2
    public static final double kS = 0.15;
    public static final double kVelocityModifier = .002;

    // Setpoint ramp rates (RPM per 20ms cycle)
    // At 200 RPM/cycle spin-up: 0 → 5000 RPM takes ~0.5 seconds
    // At 400 RPM/cycle spin-down: 5000 → 0 RPM takes ~0.25 seconds
    public static final double kRampUpRate = 200.0;
    public static final double kRampDownRate = 400.0;

    // Flywheel is "ready" when within this tolerance of target RPM
    public static final double kReadyToleranceRPM = 200;
  }

  public static final class HoodConstants {
    public static final double kEncoderOffset = 1.35; // in hood/motor rotations

    public static final double kP = 0.3;
    public static final double kI = 0.;
    public static final double kD = 5.;
    public static final double kG = 0.4; // 0.4
    public static final double kS = 0.2; // 0.2
    // public static final double kV = 0;
    // public static final double kA = 0;

    public static final double kEncoderGearRatio = 40. / 18;
    public static final double kMotorGearRatio = 12;
  }

  public static final class IntakeConstants {
    // Shooter speeds, RPM
    public static final double kVelocityLow = 500;
    public static final double kVelocityMedium = 1000;
    public static final double kVelocityHigh = 1500;
    public static final double kVelocityMax = 2500;

    public static final double kP = 0.7;
    public static final double kI = 0.;
    public static final double kD = 1.5;
    // TODO: TUNE ON ROBOT — kG gravity FF disabled because getIntakeAngle() doesn't return
    // the true physical angle. To re-enable: measure encoder position when arm is horizontal,
    // subtract that offset in getIntakeAngle(), then tune kG starting at ~0.1
    public static final double kG = 0.0;
    public static final double kS = 0.0;
    // public static final double kV = 0;
    // public static final double kA = 0;

    public static final double kMaxDeployPosition = 0.8;
    public static final double kSafeDeployPosition = 0.7;
    public static final double kMinDeployPosition = 0.625;

    // TODO: MEASURE ON ROBOT — hold arm perfectly horizontal, read IntakeDeploy/Position from NT
    public static final double kEncoderHorizontalOffset = 0.0;

    public static final double kDeployGearRatio = 18. / 22;
    public static final double kIntakeGearRatio = 1;
  }

  public static final class ClimberConstants {
    // Climber position in motor rotations
    // TODO: TUNE ON ROBOT — measure actual travel limits
    public static final double kClimberMaxPosition = 100.0;
    public static final double kClimberMinPosition = -1.0;

    public static final double kClimberGearRatio = (28. / 11.) * 125.;
  }

  public static final class VisionConstants {
    // Camera names — must match the names configured in the Limelight web interface.
    public static final String kLimelightFrontName = "limelight-front";
    public static final String kLimelightBackName = "limelight-back";

    // Front camera mount offsets (TODO: measure on real robot to nearest mm)
    public static final double kFrontCamForwardM = 0.30;
    public static final double kFrontCamLeftM = 0.0;
    public static final double kFrontCamUpM = 0.25;
    public static final double kFrontCamPitchDeg = 15.0;

    // Back camera mount offsets (TODO: measure on real robot to nearest mm)
    public static final double kBackCamForwardM = -0.30;
    public static final double kBackCamLeftM = 0.0;
    public static final double kBackCamUpM = 0.25;
    public static final double kBackCamPitchDeg = 15.0;

    // --- Rejection thresholds ---
    public static final double kMaxYawRateDegPerSec = 270.0;
    public static final double kMaxAmbiguitySingleTag = 0.5;
    public static final double kMaxPoseJumpMultiTag = 0.75;
    public static final double kMaxPoseJumpSingleTag = 0.5;
    public static final double kMaxPoseJumpSettling = 1.5;
    public static final double kMaxOdometryJumpM = 1.0;
    public static final double kMaxOdometryJumpSettlingM = 2.0;
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
    public static final int kImuSettleCycles = 25;
    public static final int kPostSettleGraceCycles = 5;

    public static final double kVisionHealthyTimeoutSec = 0.5;
    public static final double kTagDistanceStaleSec = 0.08;
  }
  
  public static final class OperatorConstants {
    // Controller and button constants

    // Drive controller port numbers
    public static final int kDriverControler = 0;
    public static final int kSecondaryDriverControler = 1;

    public static final double kStickDeadband = 0.05;
    public static final double kPathplannerDeadband = 0.01;
    public static final double kTriggerDeadband = 0.05;
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

    // Hood 550s
    public static final int kHoodPrimaryCanId = 15;
    public static final int kHoodSecondaryCanId = 16;

    // Intake Vortex
    public static final int kIntakeCanId = 17;

    // Intake Deployment 550
    public static final int kIntakeDeployCanId = 18;

    // Climber Vortexes
    public static final int kClimberPrimaryCanId = 19;
    public static final int kClimberSecondaryCanId = 20;

    // PDH
    public static final int kPDHCanId = 63;
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
      public static final String kXYStdDev = "XYStdDev";
      public static final String kAccepted = "Accepted";
      public static final String kRejectReason = "RejectReason";
      public static final String kLatencyMs = "LatencyMs";
      public static final String kPoseJumpMeters = "PoseJumpMeters";
      public static final String kHeadingDeviationDeg = "HeadingDeviationDeg";

      // System-level keys
      public static final String kTotalTagCount = "TotalTagCount";
      public static final String kVisionHealthy = "VisionHealthy";
      public static final String kImuPhase = "ImuPhase";
    }

    public static final class Shooter {
      public static final String kTable = "Shooter";
      public static final String kVelocityRPM = "Velocity RPM";
      public static final String kTargetRPM = "Target RPM";
      public static final String kRampedSetpoint = "Ramped Setpoint RPM";
      public static final String kPrimaryCurrent = "Primary Current";
      public static final String kSecondaryCurrent = "Secondary Current";
    }

    public static final class Hood {
      public static final String kTable = "Hood";
      public static final String kVelocityRPM = "Velocity RPM";
      public static final String kPositionRotations = "Position Rotations";
      public static final String kTargetRotations = "Target Position Rotations";
      public static final String kAngleDegrees = "Angle Degrees";
      public static final String kPrimaryCurrent = "Primary Current";
      public static final String kSecondaryCurrent = "Secondary Current";
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
      public static final String kDeployCurrentAmps = "Deploy Current Amps";
    }

    public static final class Climber {
      public static final String kTable = "Climber";
      public static final String kVelocityRPM = "Velocity RPM";
      public static final String kPositionRotations = "Position Rotations";
      public static final String kPrimaryCurrent = "Primary Current Amps";
      public static final String kSecondaryCurrent = "Secondary Current Amps";

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

    public static final List<Translation2d> trenchPoses = List.of(
      new Translation2d(4.620, 7.430),
      new Translation2d(4.620, 0.630),
      new Translation2d(11.93, 7.430),
      new Translation2d(11.93, 0.630)
    );

    // TODO: Populate with actual bump field coordinates
    public static final List<Translation2d> bumpPoses = List.of();
  }

   public final static class CurrentConstants
  {
  
    public static final int AMP80 = 80;
    public static final int AMP60 = 60;
    public static final int AMP40 = 40;
    public static final int AMP30 = 30;
    public static final int AMP25 = 25;
    public static final int AMP20 = 20;
    public static final int AMP15 = 15;

  }

  public static final class FuelPathConstants {
    // Intake roller speeds (RPM)
    public static final double kIntakeInSlow = 2000;
    public static final double kIntakeInStandard = 4000;
    public static final double kIntakeInFast = 6000;
    public static final double kIntakeOutSlow = -1500;
    public static final double kIntakeOutStandard = -3000;
    public static final double kIntakeOutFast = -5000;

    // Conveyor speeds (RPM)
    public static final double kConveyorIn = 500;
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
