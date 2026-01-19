package frc.robot;

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

    // Hood 550s
    public static final int kHoodPrimaryCanId = 14;
    public static final int kHoodSecondaryCanId = 15;

    // Conveyor Vortex
    public static final int kConveyorCanId = 16;

    // Intake Vortex
    public static final int kIntakeCanId = 17;

    // Intake Deployment Vortexes 
    public static final int kDeploymentPrimaryCanId = 18;
    public static final int kDeploymentSecondaryCanId = 19;

    // Lidars
    public static final int kLidarCanId = 25;

    // PDH
    public static final int kPDHCanId = 63;
  }

  public static final class NetworkTableNames {
      
    public static final class Odometry {
      public static final String kOdometry = "Odometry";
      public static final String kRobotPose2d = "Robot Pose 2d";
      public static final String kRobotRotation3d = "Robot Pitch, Roll, Yaw Rotations";
      public static final String kRobotAngularVelocity3d = "Robot Pitch, Roll, Yaw Velocities";
      public static final String kRobotVelocity = "Robot X, Y, Z Velocities";
    }

    public static final class Vision {
      public static final String kVision = "Vision";
      public static final String kVisionEstimatePose2d = "Vision Estimate Pose 2d";
      public static final String kVisionEstimateTimestamp = "Vision Estimate Timestamp";
    }

    public static final class Shooter {
      public static final String kShooter = "Shooter";
      public static final String kVelocityRPM = "Velocity RPM";
    }

    public static final class Hood {
      public static final String kHood = "Hood";
      public static final String kVelocityRPM = "Velocity RPM";
    }

    public static final class Conveyor {
      public static final String kConveyor = "Conveyor";
      public static final String kVelocityRPM = "Velocity RPM";
    }

    public static final class Intake {
      public static final String kIntake = "Intake";
      public static final String kVelocityRPM = "Velocity RPM";
    }

    public static final class IntakeDeployment {
      public static final String kIntakeDeployment = "IntakeDeployment";
      public static final String kVelocityRPM = "Velocity RPM";
    }

    public static final class Climber {
      public static final String kClimber = "Climber";
      public static final String kVelocityRPM = "Velocity RPM";
    }
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
