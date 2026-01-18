// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  SparkFlex intakeMotor = new SparkFlex(Constants.CanIdConstants.kIntakeCanId, MotorType.kBrushless);

  SparkFlex intakeDeploymentPrimaryMotor = new SparkFlex(Constants.CanIdConstants.kDeploymentPrimaryCanId, MotorType.kBrushless);
  SparkFlex intakeDeploymentSecondaryMotor = new SparkFlex(Constants.CanIdConstants.kDeploymentSecondaryCanId, MotorType.kBrushless);

  RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
  RelativeEncoder intakeDeploymentEncoder = intakeDeploymentPrimaryMotor.getEncoder();

  NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  NetworkTable intakeTable = networkTable.getTable(Constants.NetworkTableNames.Intake.kIntake);
  NetworkTable intakeDeploymentTable = networkTable.getTable(Constants.NetworkTableNames.IntakeDeployment.kIntakeDeployment);

  public IntakeSubsystem() {

  }

  /**
   * @param Velocity is in RPM
   */
  public void setIntakeVelocity(double velocity) {
    intakeMotor.set(velocity);
  }

  /**
   * @return Velocity in RPM
   */
  public double getShooterVelocity() {
    return intakeEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
