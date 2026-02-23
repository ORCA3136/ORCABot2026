// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ClimberConfigs;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.NetworkTableNames;


/*
 * 
 * This subsystem is for climbing the tower
 * Including:
 *    Managing any hooks 
 *    Managing any elevators/climbing poles
 * 
 */


public class ClimberSubsystem extends SubsystemBase {

  final SparkFlex climberPrimaryMotor = new SparkFlex(CanIdConstants.kClimberPrimaryCanId, MotorType.kBrushless);
  final SparkFlex climberSecondaryMotor = new SparkFlex(CanIdConstants.kClimberSecondaryCanId, MotorType.kBrushless);

  final RelativeEncoder climberEncoder = climberPrimaryMotor.getEncoder();

  final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  final NetworkTable climberTable = networkTable.getTable(NetworkTableNames.Climber.kTable);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    climberPrimaryMotor.configure(ClimberConfigs.climberPrimaryMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climberSecondaryMotor.configure(ClimberConfigs.climberSecondaryMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setClimberVelocity(double velocity) {
    climberPrimaryMotor.set(velocity / 6500);
  }

  public double getMotorVelocity() {
    return climberEncoder.getVelocity();
  }

  public double getClimberVelocity() {
    return climberEncoder.getVelocity() / ClimberConstants.kClimberGearRatio;
  }

  public double getMotorRotations() {
    return climberEncoder.getPosition();
  }

  public double getClimberPosition() {
    return climberEncoder.getPosition() / ClimberConstants.kClimberGearRatio;
  }

  public double getClimberPrimaryCurrent() {
    return climberPrimaryMotor.getOutputCurrent();
  }

  public double getClimberSecondaryCurrent() {
    return climberSecondaryMotor.getOutputCurrent();
  }

  public void updateNetworkTable() {
    climberTable.getEntry(NetworkTableNames.Climber.kVelocityRPM)
      .setNumber(getClimberVelocity());
    climberTable.getEntry(NetworkTableNames.Climber.kPositionRotations)
      .setNumber(getClimberPosition());
    climberTable.getEntry(NetworkTableNames.Climber.kPrimaryCurrent)
      .setNumber(getClimberPrimaryCurrent());
    climberTable.getEntry(NetworkTableNames.Climber.kSecondaryCurrent)
      .setNumber(getClimberSecondaryCurrent());
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    updateNetworkTable();
  }

  /** This method will be called once per scheduler run during simulation */
  @Override
  public void simulationPeriodic() {

  }
}
