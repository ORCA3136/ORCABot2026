// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean devMode = false;

  public static enum Reef {
    left,
    right,
    forward;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = 0.05;
    public static final double kTriggerDeadband = 0.05;
  }

  public static final class SparkConstants {
    // SPARK MAX CAN IDs

    // Drive Neos
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearLeftDrivingCanId = 6;
    public static final int kRearRightDrivingCanId = 8;

    //Drive 550s
    public static final int kFrontLeftTurningCanId = 1;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearLeftTurningCanId = 5;
    public static final int kRearRightTurningCanId = 7;

    // Elevator 
    public static final int kLeftElevatorCanId = 10;
    public static final int kRightElevatorCanId = 11;

    // Wrist
    public static final int kWristCanId = 20;

    // Intake
    public static final int kIntakeCanId = 21;

    // Climber
    public static final int kClimberCanId = 30;
    public static final int kFunnelCanId = 31;
  }

  public static final class DriveConstants {

    // Not used
    public static final boolean kGyroReversed = false;

    // Chassis configuration
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.5);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

  }

  public static final class ElevatorConstants {


    public static final class ElevatorSetpoints { // native units; our range is [0 96], so a percent
      public static final double kFeederStation = 0;
      public static final double kLevel1 = 0;
      public static final double kLevel2 = 11.3; //
      public static final double kLevel3 = 27.5;
      public static final double kLevel4 = 56; // max = 61
      public static final double kBarge = 60;

      public static final double kBottomAlgae = 14.6; // 
      public static final double kTopAlgae = 32; // 
      public static final double kProcessor = 3.6;  // 

      public static final double kElevatorSlowdownThreshhold = 40;
    }
    public static final class ElevatorPowerLevels {
      public static final double kDown = -0.1;
      public static final double kUp = 0.15;
    }

    /*
    public static final class ElevatorPIDConstants
    {
      // FOR THE PROFILED MOTION
      public static final double kElevatorKp = 0.2;
      public static final double kElevatorKi = 0;
      public static final double kElevatorKd = 0.5;
      public static final double kMaxVelocity = .2; // was 120
      public static final double kMaxAcceleration = .1; // was 500

      // FOR THE FEED FORWARD
      public static final double kElevatorkS = 0;
      public static final double  kElevatorkG = 0.0003;
      public static final double kElevatorkV = 0;
      public static final double kElevatorkA = 0;
      public static final Constraints kElevatorConstraints = new Constraints(kMaxVelocity, kMaxAcceleration);
    }
    */
  }

  public static final class WristConstants {

    public static final double wristOffset = 3.5;

    public static final class WristSetpoints { // degrees
      public static final double unblock = 25;
      public static final int kFeederStation = 4;
      public static final int kLevel1 = 4;
      public static final int kLevel2 = 30;
      public static final int kLevel3 = 25;
      public static final int kLevel4 = 53;
      public static final int kBarge = 25;
      public static final int kProcessor = 183;
      public static final int kAlgae = 132;
      public static final int kClimb = 17;
    }

    public static final class WristPowerLevels {
      public static final double kOut = 0.4;
      public static final double kIn = -0.4;
    }

    /*
    public static final class WristPIDConstants
    {
      // FOR THE PROFILED MOTION
      public static final double kWristKp = 0.008; // might need to lower; oscilates without weight sometimes, rerolls when jiggeled
      public static final double kWristKi = 0;
      public static final double kWristKd = 0;
      public static final double kMaxVelocity = 1;
      public static final double kMaxAcceleration = 1;
      public static final Constraints kWristConstraints = new Constraints(kMaxVelocity, kMaxAcceleration);

      // FOR THE FEED FORWARD
      public static final double kWristkS = 0;
      public static final double kWristkG = 0;
      public static final double kWristkV = 0;
      public static final double kWristkA = 0;
    }
    */
  }

  public static final class WallConstants { // constants for the "walls" in elevator motion   most likely redundant now
    public static final double kElevatorAboveTopBar = 54; // 90 -> 54.0
    public static final double kElevatorBelowTopBar = 13.2; // 22
    public static final double kElevatorAboveBottomBar = 0;
    public static final double kElevatorBelowBottomBar = 3; //5
    // public static final double kMysteryPos = 12; //20


  }

  public static final class IntakeConstants {
    
    public static final class IntakePowerLevels {
      public static final double kOut = -0.9;
      public static final double kFeed = -0.6;  // Algae out
      public static final double kIn = 0.9;
      // public static final double kVomit = 0.95; // Algae in

      public static final double kAlgaeIn = 0.45; // 0.45
      public static final double kAlgaeHold = 0.1; // 0.1
      public static final double kAlgaeOut = -0.95; // -0.95
    }
  }

  public static final class ClimberConstants {
    
    public static final double kFunnelSpeed = -0.3;
    public static final double kClimberInSpeed = -1;
    public static final double kClimberOutSpeed = 1;

    public static final double kClimberInPos = -60;
    public static final double kClimberOutPos = 192;
    public static final double kFunnelOutPos = -40;
  }

  public static final class Limits {

    // public static final double kElevatorSafetyThreshold = 5.0;

    public static final double kElevatorMaxHeight = 61;     //0.948;
    public static final double kElevatorMinHeight = 0.0;


    // public static final double kWristSafetyThreshold = 25; // [20, 30]


    public static final double kWristMinAngle = WristConstants.wristOffset + 1; // degrees
    public static final double kWristMaxAngle = 200; // degrees  110 untested; was 94

    public static final double MAX_SPEED = Units.feetToMeters(10); // theoretical: 14.63 Ft/s
    public static final double ELEVATOR_SPEED_FACTOR = 0.5; // Used to scale drive speed when elevator is above the threshold
    public static final double MEDIUM_SPEED_FACTOR = 0.42;
    public static final double PATHPLANNER_MAX_SPEED = Units.feetToMeters(5);
  }

  public static final class PathPlannerConstants {

    public static final PathConstraints testingConstraints = new PathConstraints(
        Units.feetToMeters(1.5), 2.0,             
        Units.degreesToRadians(50), Units.degreesToRadians(300));

    public static final PathConstraints slowConstraints = new PathConstraints(
        Units.feetToMeters(3.5), 4.0,             
        Units.degreesToRadians(100), Units.degreesToRadians(720));

    public static final PathConstraints defaultConstraints = new PathConstraints(
        Units.feetToMeters(8), 4.0,
        Units.degreesToRadians(200), Units.degreesToRadians(720));

    public static final PathConstraints fastConstraints = new PathConstraints(
      Units.feetToMeters(14), 4.0,
        Units.degreesToRadians(360), Units.degreesToRadians(720));
  }

  /*
  public static final class PhysicalConstants {
    public static final double elevatorSupportBar = 34;
  }
  */

  public static final class FieldPoses {

    public static final double[] fieldSize = {17.55, 8.05};
    // Wall thickness is 0.051
    public static final Pose2d blueCenterOfReef = new Pose2d(4.487, 4.025, new Rotation2d()); // blue
    public static final Pose2d redCenterOfReef = new Pose2d(13.065, 4.025, new Rotation2d()); // red

    public static final List<Pose2d> blueReefPoses = new ArrayList<Pose2d>(){{
      add(new Pose2d(2.890, 4.025, new Rotation2d(Units.degreesToRadians(0))));
      add(new Pose2d(3.689, 2.642, new Rotation2d(Units.degreesToRadians(60.0))));
      add(new Pose2d(5.285, 2.642, new Rotation2d(Units.degreesToRadians(120.0))));
      add(new Pose2d(6.087, 4.025, new Rotation2d(Units.degreesToRadians(180.0))));
      add(new Pose2d(5.285, 5.408, new Rotation2d(Units.degreesToRadians(240.0))));
      add(new Pose2d(3.689, 5.408, new Rotation2d(Units.degreesToRadians(300.0))));
    }};

    public static final List<Pose2d> redReefPoses = new ArrayList<Pose2d>(){{
      add(new Pose2d(11.466, 4.025, new Rotation2d(Units.degreesToRadians(0))));
      add(new Pose2d(12.265, 2.642, new Rotation2d(Units.degreesToRadians(60.0))));
      add(new Pose2d(13.861, 2.642, new Rotation2d(Units.degreesToRadians(120.0))));
      add(new Pose2d(14.663, 4.025, new Rotation2d(Units.degreesToRadians(180.0))));
      add(new Pose2d(13.861, 5.408, new Rotation2d(Units.degreesToRadians(240.0))));
      add(new Pose2d(12.265, 5.408, new Rotation2d(Units.degreesToRadians(300.0))));
    }};

    public static final double leftOffset = 0.165;
    public static final double L2ScoringOffset = 0.285;
    public static final double L3ScoringOffset = 0.145;  // Was 1.55
    public static final double L4ScoringOffset = 0.315;
    public static final double topAlgaeScoringOffset = 0.23;
    public static final double bottomAlgaeScoringOffset = 0.25;

    public static final double generalScoringOffset = 0.285;

    public static final double reefElevatorRange = 2;  // Units.feetToMeters(6); // 1.8 meters
    public static final double reefAlgaeElevatorRange = 2.4;
    public static final double reefAutoElevatorRange = 3;
  }
 
  public static final class Colors {
    //These are all the led optios, if you want more you will have to go to a REV website called "LED BLINKIN DRIVER"

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
