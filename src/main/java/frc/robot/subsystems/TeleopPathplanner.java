// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  NetworkTable odometryTable = networkTable.getTable(NetworkTableNames.Odometry.kTable);

  StructSubscriber<Pose2d> robotPositionSubscriber = odometryTable
      .getStructTopic(NetworkTableNames.Odometry.kRobotPose2d, Pose2d.struct).subscribe(new Pose2d(), PubSubOption.periodic(0.02));
  DoubleArraySubscriber robotVelocitySubscriber = odometryTable
      .getDoubleArrayTopic(NetworkTableNames.Odometry.kRobotVelocity).subscribe(new double[] {0,0,0}, PubSubOption.periodic(0.02));

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
  Pose2d currentPose; double[] robotVelocities;
  
  /** Creates a new TeleopPathplanner. */
  public TeleopPathplanner() 
  {
    Optional<Alliance> startingAllinace = DriverStation.getAlliance();
    if (startingAllinace.isPresent())
      startingSide = currentSide = (startingAllinace.get() == Alliance.Blue ? FieldSide.BlueSide : FieldSide.RedSide);
  }


  
  public Command createTrenchPathCommand(SwerveSubsystem m_drive) {
    return Commands.defer(() -> {
      // Find nearest trench in heading direction
      // Get the waypoint with correct heading
      // Get next waypoint for after the trench

      // Make the waypoint list
      // Make the path
      // Make the command

      // TODO: Implement trench path generation
      return Commands.none();
    }, Set.of(m_drive));
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

  public boolean haveConditionsChanged() {
    return false;
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    currentPose = robotPositionSubscriber.get();
    robotVelocities = robotVelocitySubscriber.get();

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