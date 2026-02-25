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
import edu.wpi.first.networktables.NetworkTableEntry;
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

  // Cached NetworkTable entries — avoids hash lookups every cycle (50Hz)
  private final NetworkTableEntry velocityEntry = kickerTable.getEntry(NetworkTableNames.Kicker.kVelocityRPM);
  private final NetworkTableEntry currentEntry = kickerTable.getEntry(NetworkTableNames.Kicker.kCurrentAmps);

  /** Creates a new KickerSubsystem. */
  public KickerSubsystem() {

    kickerMotor.configure(KickerConfigs.kickerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  /** @return Motor for simulation access */
  public SparkFlex getMotor() {
    return kickerMotor;
  }

  /**
   * Sets kicker motor output. Takes an RPM-scale value and normalizes it to [-1, 1]
   * duty cycle by dividing by the NEO Vortex free speed (6500 RPM).
   * @param speed RPM-scale value (e.g. 4000 for feeding, -2000 for reverse)
   */
  public void setKickerDutyCycle(double speed) {
    kickerMotor.set(speed / RobotConstants.kNeoVortexFreeSpeedRPM);
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
    velocityEntry.setDouble(getKickerVelocity());
    currentEntry.setDouble(getKickerCurrent());
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
