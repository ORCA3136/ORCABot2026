// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldPositions;
import frc.robot.Constants.NetworkTableNames;


/*
 * 
 * This subsystem is for all on the fly pathplanner operations
 * Including:
 *    Creating paths 
 *    Creating Pathplanner commands
 * 
 */


public class TeleopPathplanner extends SubsystemBase {

  private final SwerveSubsystem swerveSubsystem;

  NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  NetworkTable odometryTable = networkTable.getTable(NetworkTableNames.Odometry.kTable);

  public enum FieldSide {
    BlueSide,
    RedSide,
    Middle,
    OutOfBounds
  }
  public enum FieldTargets {
    kHub,         // Scoring element
    kTower,       // Climbing challenge
    kOutpost,     // Human player station
    kDepot,       // Zone on the floor with staring fuel
    kBlueTrench,
    kRedTrench,
    kBlueBump,
    kRedBump
  }

  FieldSide startingSide, currentSide;
  Pose2d currentPose, targetHubPose; 
  ChassisSpeeds robotVelocities;
  double angleToHub, distanceToHub;

  
  /** Creates a new TeleopPathplanner. */
  public TeleopPathplanner(SwerveSubsystem swerve) 
  {
    swerveSubsystem = swerve;

    setStartingAlliance();
  }



  public List<Waypoint> moveToShootingPose(int listPosition) {

    // List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
    //     new Pose2d(currentPose.getTranslation(), getPathVelocityHeading(swerveSubsystem.getFieldVelocity(), waypoint))
    // );

    // Find end goal pose
    // Translation2d endGoal = FieldPositions.kBlueShootingPoses.get(listPosition);
    // Translation2d hubTranslation2d = startingSide == FieldSide.BlueSide ? 
    //             FieldPositions.kBlueFieldElements.get(0) : FieldPositions.kRedFieldElements.get(0);

    // Find angle to hub at end
    // double targetAngleToHub = Math.atan2(hubTranslation2d.getY() - endGoal.getY(), hubTranslation2d.getX() - endGoal.getX());

    // Check if in different section
    FieldSide endState = startingSide;

    if (endState == FieldSide.BlueSide) {
      if (startingSide == FieldSide.RedSide) ;
        // Add Red trench
      if (startingSide != FieldSide.BlueSide) ;
        // Add Blue trench
    }

    else if (endState == FieldSide.RedSide) {
      if (startingSide == FieldSide.BlueSide) ;
        // Add Red trench
      if (startingSide != FieldSide.RedSide) ;
        // Add Blue trench
    }



    // if (true) {
      
    //   // Add waypoints
    //   // Subtract/Add 1 to
    //   return null;
    // }
      
      

    
    
    /*
     * Make list of waypoints
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(m_drive.getPose().getTranslation(), getPathVelocityHeading(m_drive.getFieldVelocity(), waypoint)),
            waypoint
        );
     */

    // Create path command



    return null;
  }

  public double hubRotation() {
    if (targetHubPose == null) return 0;
    Pose2d targetPose2d = targetHubPose;
    Pose2d currentPose2d = swerveSubsystem.getPose();
    
    // Assuming angular velocity is negligable - it might not be negligable
    // if (robotVelocities.vxMetersPerSecond + robotVelocities.vyMetersPerSecond > 0.05) {
    //   // Based on the robot speeds only
    //   targetPose2d = new Pose2d(targetHubPose.getX() - robotVelocities.vxMetersPerSecond, 
    //                             targetHubPose.getY() - robotVelocities.vyMetersPerSecond, new Rotation2d());
    // }

    double targetAngle = Math.atan2(currentPose2d.getY() - targetPose2d.getY(), currentPose2d.getX() - targetPose2d.getX());

    // Convert angle to speed

    DataLogManager.log("Hub Rotation: current rotation radians " + targetAngle);
  
    return targetAngle;
  }

  public void updateCurrentSide() {
    double robotX = currentPose.getX();

    if (robotX < 0 || robotX > 16.56)
      currentSide = FieldSide.OutOfBounds;
    else if (robotX < 4)
      currentSide = FieldSide.BlueSide;
    else if (robotX > 12.54)
      currentSide = FieldSide.RedSide;
    else 
      currentSide = FieldSide.Middle;
  }

  public void setStartingAlliance() {
    Optional<Alliance> startingAllinace = DriverStation.getAlliance();
    if (startingAllinace.isPresent())
      startingSide = currentSide = (startingAllinace.get() == Alliance.Blue ? FieldSide.BlueSide : FieldSide.RedSide);

    if (startingSide == FieldSide.RedSide)
      targetHubPose = new Pose2d(FieldPositions.kRedFieldElements.get(0), new Rotation2d());
    else 
      targetHubPose = new Pose2d(FieldPositions.kBlueFieldElements.get(0), new Rotation2d());
  }

  public void updateHubTargets() {
    angleToHub = Math.atan2(currentPose.getY() - targetHubPose.getY(), currentPose.getX() - targetHubPose.getX());
    distanceToHub = Math.pow(Math.pow(currentPose.getY() - targetHubPose.getY(), 2) + Math.pow(currentPose.getX() - targetHubPose.getX(), 2), 0.5);
  }

  public Command getStartingAlliance() {
    return Commands.runOnce(() -> {
      Optional<Alliance> startingAllinace = DriverStation.getAlliance();
      if (startingAllinace.isPresent())
        startingSide = currentSide = (startingAllinace.get() == Alliance.Blue ? FieldSide.BlueSide : FieldSide.RedSide);

      if (startingSide == FieldSide.RedSide)
        targetHubPose = new Pose2d(FieldPositions.kRedFieldElements.get(0), new Rotation2d());
      else 
        targetHubPose = new Pose2d(FieldPositions.kBlueFieldElements.get(0), new Rotation2d());
      });
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    currentPose = swerveSubsystem.getPose();
    robotVelocities = swerveSubsystem.getFieldVelocity();

    updateHubTargets();
    updateCurrentSide();
  }
}


/*
 * 
 * Poses - Need correct headings
 *  - Trenches - 4
 *  - Depot
 *  - Outpost
 *  - Climb
 * 
 * Trench
 *  - 50.34 in wide (50 in)
 *  - Robot width 34.5 in
 *  - Robot length 42.5 in (With intake down)
 *  - Intake 8.4 in
 * 
 * Left Trigger
 *  - Navigate through nearest trench in heading
 * Right Trigger
 *  - Auto shoot while moving
 * 
 */