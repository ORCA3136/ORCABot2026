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

  SparkFlex intakeDeployPrimaryMotor = new SparkFlex(Constants.CanIdConstants.kDeploymentPrimaryCanId, MotorType.kBrushless);
  SparkFlex intakeDeploySecondaryMotor = new SparkFlex(Constants.CanIdConstants.kDeploymentSecondaryCanId, MotorType.kBrushless);

  RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
  RelativeEncoder intakeDeployEncoder = intakeDeployPrimaryMotor.getEncoder();

  NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  NetworkTable intakeTable = networkTable.getTable(Constants.NetworkTableNames.Intake.kIntake);
  NetworkTable intakeDeployTable = networkTable.getTable(Constants.NetworkTableNames.IntakeDeployment.kIntakeDeployment);

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
  public double getIntakeVelocity() {
    return intakeEncoder.getVelocity();
  }

  /**
   * @param Velocity is in RPM
   */
  public void setIntakeDeployVelocity(double velocity) {
    intakeMotor.set(velocity);
  }

  /**
   * @return Velocity in RPM
   */
  public double getIntakeDeployVelocity() {
    return intakeEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeTable.getEntry(Constants.NetworkTableNames.Intake.kVelocityRPM)
      .setNumber(getIntakeVelocity());
    intakeDeployTable.getEntry(Constants.NetworkTableNames.IntakeDeployment.kVelocityRPM)
      .setNumber(getIntakeDeployVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
