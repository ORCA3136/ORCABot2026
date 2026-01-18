// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

/*
 * 
 * This subsystem is for all vision operations
 * Including:
 *    Limelight 
 *    Any distance or beam break sensors
 * 
 */

public class VisionSubsystem extends SubsystemBase {

  String limelightOne = "limelight-left";
  String limelightTwo = "limelight-right";

  private LaserCan lidar;
  private LaserCan.Measurement intakeLidar;

  private boolean intakeStatus;

  public VisionSubsystem() {
    // name of constant may need to change
    lidar = new LaserCan(Constants.CanIdConstants.kLidarCanId);

  }

  /**
   * @return True if limelightOne sees a valid tag
   */
  public boolean getLeftTV() {
    return LimelightHelpers.getTV("limelight-left");
  }

  /**
   * @return True if limelightTwo sees a valid tag
   */
  public boolean getRightTV() {
    return LimelightHelpers.getTV("limelight-right");
  }

  /**
   * @return True if any limelight sees a valid tag
   */
  public boolean getTV() {
    return getLeftTV() || getRightTV();
  }

  /**
   * @return The distance before the lidar sees something
   */
  public double getLidarMeasurement() {
    Measurement it = lidar.getMeasurement();

    NetworkTableInstance.getDefault().getTable("Lidar").getEntry("Lidar status").setDouble(it.status);
    
    if (it == null) 
      return 999999;
    return it.distance_mm;
  }





  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


    NetworkTableInstance.getDefault().getTable("Lidar").getEntry("Lidar distance").setDouble(getLidarMeasurement());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}