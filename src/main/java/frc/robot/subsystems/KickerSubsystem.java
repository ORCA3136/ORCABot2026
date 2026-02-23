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


public class KickerSubsystem extends SubsystemBase {

  final SparkFlex kickerMotor = new SparkFlex(CanIdConstants.kKickerCanId, MotorType.kBrushless);

  final RelativeEncoder kickerEncoder = kickerMotor.getEncoder();

  final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  final NetworkTable kickerTable = networkTable.getTable(NetworkTableNames.Kicker.kTable);

  /** Creates a new ConveyorSubsystem. */
  public KickerSubsystem() {

    kickerMotor.configure(KickerConfigs.kickerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  /** @return Motor for simulation access */
  public SparkFlex getMotor() {
    return kickerMotor;
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

  /**
   * @return Current is in Amps
   */
  public double getKickerCurrent() {
    return kickerMotor.getOutputCurrent();
  }

  public void updateNetworkTable() {
    kickerTable.getEntry(NetworkTableNames.Kicker.kVelocityRPM)
      .setNumber(getKickerVelocity());
    kickerTable.getEntry(NetworkTableNames.Kicker.kCurrentAmps)
      .setNumber(getKickerCurrent());
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
