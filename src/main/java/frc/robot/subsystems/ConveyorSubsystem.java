// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.*;
import frc.robot.Constants.*;


/*
 * 
 * This subsystem is for fuel interactions
 * Including:
 *    Intaking/outaking fuel from the floor
 *    Transporting fuel to the shooter
 * 
 */


public class ConveyorSubsystem extends SubsystemBase {

  final SparkFlex conveyorMotor = new SparkFlex(CanIdConstants.kConveyorCanId, MotorType.kBrushless);
  final SparkFlex kickerMotor = new SparkFlex(CanIdConstants.kKickerCanId, MotorType.kBrushless);

  final RelativeEncoder conveyorEncoder = conveyorMotor.getEncoder();
  final RelativeEncoder kickerEncoder = kickerMotor.getEncoder();

  final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  final NetworkTable conveyorTable = networkTable.getTable(NetworkTableNames.Conveyor.kTable);
  final NetworkTable kickerTable = networkTable.getTable(NetworkTableNames.Kicker.kTable);

  /** Creates a new ConveyorSubsystem. */
  public ConveyorSubsystem() {

    conveyorMotor.configure(ConveyorConfigs.conveyorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    kickerMotor.configure(ConveyorConfigs.kickerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  /**
   * @param Velocity is in RPM
   */
  public void setConveyorVelocity(double velocity) {
    conveyorMotor.set(velocity / 6500);
  }

  /**
   * @return Velocity in RPM
   */
  public double getConveyorVelocity() {
    return conveyorEncoder.getVelocity();
  }

   /**
   * @param Velocity is in RPM
   */
  public void setKickerVelocity(double velocity) {
    kickerMotor.set(velocity / 6500);
  }

  /**
   * @return Velocity in RPM
   */
  public double getKickerVelocity() {
    return kickerEncoder.getVelocity();
  }

  public void updateNetworkTable() {
    conveyorTable.getEntry(NetworkTableNames.Conveyor.kVelocityRPM)
      .setNumber(getConveyorVelocity());
    kickerTable.getEntry(NetworkTableNames.Kicker.kCurrent)
      .setNumber(getConveyorVelocity());
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
