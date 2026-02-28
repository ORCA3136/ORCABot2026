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
import frc.robot.RobotLogger;


/*
 * 
 * This subsystem is for fuel interactions
 * Including:
 *    Intaking/outaking fuel from the floor
 *    Transporting fuel to the shooter
 * 
 */


public class KickerSubsystem extends SubsystemBase {

  // Shot detection: log once when kicker spins up past threshold, reset when it drops
  private static final double SHOT_VELOCITY_THRESHOLD = 500.0; // RPM
  private boolean shotDetected = false;

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

  /** @return if the kicker is stalling */
  public boolean isMotorStalling() {
    double currentDraw = getKickerCurrent();
    double motorVelocity = getKickerVelocity();
    // double motorAcceleration = kickerMotor.get
    return false;
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

    // Shot detection: log once per shot when kicker exceeds threshold
    double velocity = getKickerVelocity();
    if (velocity > SHOT_VELOCITY_THRESHOLD && !shotDetected) {
      shotDetected = true;
      RobotLogger.logShot();
    } else if (velocity < SHOT_VELOCITY_THRESHOLD) {
      shotDetected = false;
    }
  }

  /** This method will be called once per scheduler run during simulation */
  @Override
  public void simulationPeriodic() {

  }
}
