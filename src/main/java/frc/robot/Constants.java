package frc.robot;

import edu.wpi.first.wpilibj.RobotController;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {



  // Robot max speed
  public static final class RobotConstants {
    public static final double kDriveMaxSpeedMps = 3.05; // Meeters per second
    public static final double kDriveMaxSpeedFps = 10; // Feet per second
  }



  public static final class ShooterConstants {
    // Shooter speeds, RPM
    public static final double kVelocityLow = 2000;
    public static final double kVelocityMedium = 3500;
    public static final double kVelocityHigh = 5000;
    public static final double kVelocityMax = 6500;
  }

  public static final class HoodConstants {
    public static final double kP = 0.1;
    public static final double kI = 0.;
    public static final double kD = 1.5;
    public static final double kG = 0.5;
    public static final double kS = 0.;
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

    public static final double kP = 0.1;
    public static final double kI = 0.;
    public static final double kD = 1.5;
    public static final double kG = 0.5;
    public static final double kS = 0.;
    // public static final double kV = 0;
    // public static final double kA = 0;

    public static final double kDeployGearRatio = 15. / 36;
    public static final double kIntakeGearRatio = 1;
  }

  public static final class VisionConstants {
    public static final String limelightOneName = "limelight-left";
    public static final String limelightTwoName = "limelight-right";
  }
  
  public static final class OperatorConstants {
    // Controller and button constants

    // Drive controller port numbers
    public static final int kDriverControler = 0;
    public static final int kSecondaryDriverControler = 1;

    public static final double kStickDeadband = 0.05;
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

    // Intake Deployment Vortex
    public static final int kIntakeDeployCanId = 18;

    // Climber Vortexes
    public static final int kClimberPrimaryCanId = 19;
    public static final int kClimberSecondaryCanId = 20;

    // Lidars
    public static final int kLidarCanId = 25;

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
      public static final String kVisionEstimatePose2d = "Vision Estimate Pose 2d";
      public static final String kVisionEstimateTimestamp = "Vision Estimate Timestamp";
    }

    public static final class Shooter {
      public static final String kTable = "Shooter";
      public static final String kVelocityRPM = "Velocity RPM";
      public static String kPrimaryCurrent = "Primary Current";
    public static String kSecondaryCurrent = "Secondary Current";
    }

    public static final class Hood {
      public static final String kTable = "Hood";
      public static final String kVelocityRPM = "Velocity RPM";
      public static final String kPositionRotations = "Position Rotations";
      public static final String kTargetRotations = "Target Position Rotations";
    public static String kPrimaryCurrent = "Primary Current";
    public static String kSecondaryCurrent = "Secondary Current";
    }

    public static final class Conveyor {
      public static final String kTable = "Conveyor";
      public static final String kVelocityRPM = "Velocity RPM";
    }

    public static final class Intake {
      public static final String kTable = "Intake";
      public static final String kVelocityRPM = "Velocity RPM";
    }

    public static final class IntakeDeploy {
      public static final String kTable = "IntakeDeploy";
      public static final String kVelocityRPM = "Velocity RPM";
    }

    public static final class Climber {
      public static final String kTable = "Climber";
      public static final String kVelocityRPM = "Velocity RPM";
    }
  }

  public static final class FieldPositions {

    public static final List<Translation2d> kBlueFieldElements = new ArrayList<Translation2d>(){{
      add(new Translation2d(4.597,  4.035));         // Blue Hub
      add(new Translation2d(0.2,    0.666));         // Blue Outpost
      add(new Translation2d(1.065,  3.745));         // Blue Tower
      add(new Translation2d(0.410,  5.965));         // Blue Depot
    }};

    public static final List<Translation2d> kRedFieldElements = new ArrayList<Translation2d>(){{
      add(new Translation2d(11.938, 4.035));         // Red Hub
      add(new Translation2d(16.621, 7.403));         // Red Outpost
      add(new Translation2d(15.48,  3.745));         // Red Tower
      add(new Translation2d(16.13,  2.103));         // Red Depot
    }};

    public static final List<Translation2d> trenchPoses = new ArrayList<Translation2d>(){{
      add(new Translation2d(4.620, 7.430));
      add(new Translation2d(4.620, 0.630));
      add(new Translation2d(11.93, 7.430));
      add(new Translation2d(11.93, 0.630));
    }};

    public static final List<Translation2d> bumpPoses = new ArrayList<Translation2d>(){{
    }};
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
