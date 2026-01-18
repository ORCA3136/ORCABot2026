// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


/*
 * 
 * This subsystem is for fuel interactions
 * Including:
 *    Intaking/outaking fuel from the floor
 *    Transporting fuel to the shooter
 * 
 */


public class IntakeSubsystem extends SubsystemBase {
  //

  SparkFlex intakeDeploymentPrimaryMotor = new SparkFlex(Constants.CanIdConstants.kShooterPrimaryCanId, MotorType.kBrushless);
  SparkFlex intakeDeploymentSecondaryMotor = new SparkFlex(Constants.CanIdConstants.kShooterSecondaryCanId, MotorType.kBrushless);

  // encoder(S)

  public IntakeSubsystem() {}

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
