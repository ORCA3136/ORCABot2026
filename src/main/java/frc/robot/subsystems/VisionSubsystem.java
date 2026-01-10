// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  public VisionSubsystem() {
    
  }

  public boolean getLeftTV() {
    return LimelightHelpers.getTV("limelight-left");
  }
  public boolean getRightTV() {
    return LimelightHelpers.getTV("limelight-right");
  }
  public boolean getTV() {
    return getLeftTV() || getRightTV();
  }

  // public void updatePoseEstimator(SwerveDrive swerve) {

  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
