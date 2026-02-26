// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.*;
import frc.robot.Constants.*;


/**
 * This subsystem Manages the HoodPID
 */


public class HoodSubsystem extends SubsystemBase {

  double rotations;
  boolean hoodMovingForward = true; // true is positive
  
  final SparkMax hoodPrimaryMotor = new SparkMax(CanIdConstants.kHoodPrimaryCanId, MotorType.kBrushless);
  final SparkMax hoodSecondaryMotor = new SparkMax(CanIdConstants.kHoodSecondaryCanId, MotorType.kBrushless);

  final AbsoluteEncoder hoodEncoder = hoodPrimaryMotor.getAbsoluteEncoder();

  final SparkClosedLoopController hoodPIDController = hoodPrimaryMotor.getClosedLoopController();

  final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  final NetworkTable hoodTable = networkTable.getTable(NetworkTableNames.Hood.kTable);

  // Cached NetworkTable entries — avoids hash lookups every cycle (50Hz)
  private final NetworkTableEntry velocityEntry = hoodTable.getEntry(NetworkTableNames.Hood.kVelocityRPM);
  private final NetworkTableEntry positionEntry = hoodTable.getEntry(NetworkTableNames.Hood.kPositionRotations);
  private final NetworkTableEntry targetEntry = hoodTable.getEntry(NetworkTableNames.Hood.kTargetRotations);
  private final NetworkTableEntry angleEntry = hoodTable.getEntry(NetworkTableNames.Hood.kAngleDegrees);
  private final NetworkTableEntry primaryCurrentEntry = hoodTable.getEntry(NetworkTableNames.Hood.kPrimaryCurrent);
  private final NetworkTableEntry secondaryCurrentEntry = hoodTable.getEntry(NetworkTableNames.Hood.kSecondaryCurrent);

  /** Creates a new HoodSubsystem. */
  public HoodSubsystem() {

    hoodPrimaryMotor.configure(HoodConfigs.primaryHoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hoodSecondaryMotor.configure(HoodConfigs.secondaryHoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    updateHoodTarget(0);
  }

  /** Calculates the current hood feedforward
   * {@summary Hood feedforward includes gravitational force, static loss, air resistance, and robot acceleration} */
  public double calculateFeedForward() {
    // FF pivot = Ksta + Kvel * TarVel + Kgrav * cos(angle) + Kaccel * RobAccel * sin(angle)
    return HoodConstants.kS + HoodConstants.kG * Math.cos(getHoodAngle());
  }

  /** Sets the hood setpoint angle */
  public void setHoodPIDAngle() {
    hoodPIDController.setSetpoint(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0, calculateFeedForward());
  }

  /** Updates the rotations variable which is used in the setPIDAngle method
   * @param angle is in Degrees
   * TODO: TUNE ON ROBOT — verify this calculation matches the encoder conversion factor in Configs.java */
  public void updateHoodTarget(double angle) {
    rotations = HoodConstants.kEncoderOffset + (angle / 360) * HoodConstants.kEncoderGearRatio * HoodConstants.kMotorGearRatio;
  }

  public void increaseHoodAngle() {
    rotations += (1. / 360.) * HoodConstants.kEncoderGearRatio * HoodConstants.kMotorGearRatio;
  }

  public void decreaseHoodAngle() {
    rotations -= (1. / 360.) * HoodConstants.kEncoderGearRatio * HoodConstants.kMotorGearRatio;
  }

  public void setHoodTarget(double target) {
    rotations = target;
  }

  public double getHoodTarget() {
    return rotations;
  }

  public void changeHoodDirection() {
    hoodMovingForward = !hoodMovingForward;
  }

  public boolean getHoodMovingForward() {
    return hoodMovingForward;
  }

  /**
   * Sets hood motor output. Takes an RPM-scale value and normalizes it to [-1, 1]
   * duty cycle by dividing by the NEO 550 free speed (11000 RPM).
   * Only commands the primary motor — the secondary is a follower.
   * Note: The hood also has PID position control via setHoodPIDAngle(). Use this
   * method only for manual/testing control.
   * @param speed RPM-scale value (e.g. 3000 for forward, -3000 for reverse)
   */
  public void setHoodDutyCycle(double speed) {
    hoodPrimaryMotor.set(speed / RobotConstants.kNeo550FreeSpeedRPM);
  }

  /** @return Primary motor for simulation access */
  public SparkMax getPrimaryMotor() {
    return hoodPrimaryMotor;
  }

  /** @return Motor Rotations */
  public double getHoodMotorRotations() {
    return hoodEncoder.getPosition();
  }

  /** @return Angle in Rad */
  public double getHoodAngle() {
    return 2 * Math.PI * ((hoodEncoder.getPosition() - HoodConstants.kEncoderOffset) / (HoodConstants.kMotorGearRatio * HoodConstants.kEncoderGearRatio));
  }

  /** @return Velocity in RPM */
  public double getHoodVelocity() {
    return hoodEncoder.getVelocity();
  }

  /** @return Current in Amps */
  public double getHoodPrimaryCurrent() {
    return hoodPrimaryMotor.getOutputCurrent();
  }

  // /** @return Current in Amps */
  public double getHoodSecondaryCurrent() {
    return hoodSecondaryMotor.getOutputCurrent();
  }

  /** Publish continuous values to network table */
  public void updateNetworkTable() {
    velocityEntry.setDouble(getHoodVelocity());
    positionEntry.setDouble(getHoodMotorRotations());
    targetEntry.setDouble(rotations);
    angleEntry.setDouble(Math.toDegrees(getHoodAngle()));
    primaryCurrentEntry.setDouble(getHoodPrimaryCurrent());
    secondaryCurrentEntry.setDouble(getHoodSecondaryCurrent());
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    
    updateNetworkTable();

    setHoodPIDAngle();
  }

  /** This method will be called once per scheduler run during simulation */
  @Override
  public void simulationPeriodic() {
    
  }
}