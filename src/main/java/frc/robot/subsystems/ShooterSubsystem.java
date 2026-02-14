// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.*;
import frc.robot.Constants.*;


/*
 * 
 * This subsystem is for all Shooter operations
 * Including:
 *    logging shooter data
 *    Managing the speeds of the flywheels
 * 
 */


public class ShooterSubsystem extends SubsystemBase {

  double shooterVelocity = 0;
  double rotations;
  boolean hoodDirection = true; // true is positive
  
  final SparkFlex shooterPrimaryMotor = new SparkFlex(CanIdConstants.kShooterPrimaryCanId, MotorType.kBrushless);
  final SparkFlex shooterSecondaryMotor = new SparkFlex(CanIdConstants.kShooterSecondaryCanId, MotorType.kBrushless);

  final SparkMax hoodPrimaryMotor = new SparkMax(CanIdConstants.kHoodPrimaryCanId, MotorType.kBrushless);
  final SparkMax hoodSecondaryMotor = new SparkMax(CanIdConstants.kHoodSecondaryCanId, MotorType.kBrushless);

  final RelativeEncoder shooterEncoder = shooterPrimaryMotor.getEncoder();
  final AbsoluteEncoder hoodEncoder = hoodPrimaryMotor.getAbsoluteEncoder();

  final SparkClosedLoopController hoodPIDController = hoodPrimaryMotor.getClosedLoopController();

  final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  final NetworkTable shooterTable = networkTable.getTable(NetworkTableNames.Shooter.kTable);
  final NetworkTable hoodTable = networkTable.getTable(NetworkTableNames.Hood.kTable);


  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    shooterPrimaryMotor.configure(ShooterConfigs.primaryShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterSecondaryMotor.configure(ShooterConfigs.secondaryShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    hoodPrimaryMotor.configure(ShooterConfigs.primaryHoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hoodSecondaryMotor.configure(ShooterConfigs.secondaryHoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  /** Calculates the current hood feedforward
   * {@summary Hood feedforward includes gravitational force, static loss, air resistance, and robot acceleration} */
  public double calculateFeedForward() {
    // FF pivot = Ksta + Kvel * TarVel + Kgrav * cos(angle) + Kaccel * RobAccel * sin(angle)
    return HoodConstants.kS + HoodConstants.kG * Math.cos(getHoodAngle());
  }

  /** Sets the hood setpoint angle */
  public void setPIDAngle() {
    hoodPIDController.setSetpoint(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0, calculateFeedForward());
  }

  /** Updates the rotations variable which is used in the setPIDAngle method
   * @param angle is in Degrees */
  public void updateHoodTarget(double angle) {
    rotations = (angle / 360) * HoodConstants.kEncoderGearRatio * HoodConstants.kMotorGearRatio;
  }

  public void setHoodTarget(double target) {
    rotations = target;
  }

  public double getHoodTarget() {
    return rotations;
  }

  public void changeHoodDirection() {
    hoodDirection = !hoodDirection;
  }

  public boolean getHoodDirection() {
    return hoodDirection;
  }

  /** Increases the speed of the shooter by 350 RPM */
  public void increaseShooterVelocity() {
    shooterVelocity += 100;
    if (shooterVelocity > 6500) shooterVelocity = 6500;

    setShooterVelocity(shooterVelocity);
  }

  /** Decreases the speed of the shooter by 350 RPM */ 
  public void decreaseShooterVelocity() {
    shooterVelocity -= 100;
    if (shooterVelocity < 0) shooterVelocity = 0;
      
    setShooterVelocity(shooterVelocity);
  }
  
  /** @param Velocity is in RPM */
  public void setShooterVelocityHigh() {
    shooterPrimaryMotor.set(ShooterConstants.kVelocityHigh);
  }

  /** @param Velocity is in RPM */
  public void setShooterVelocityNone() {
    shooterPrimaryMotor.set(0);
  }

  /** @param velocity is in RPM */
  public void setShooterVelocity(double velocity) {
    shooterPrimaryMotor.set(velocity / 6500);
    shooterSecondaryMotor.set(velocity / 6500);
  }

  /** @return Velocity in RPM */
  public double getShooterVelocity() {
    return shooterEncoder.getVelocity();
  }

  /** @param Velocity is in RPM */
  public void setHoodVelocity(double velocity) {
    hoodPrimaryMotor.set(velocity / 6500);
    hoodSecondaryMotor.set(velocity / 6500);
  }

  /** @return Motor Rotations */
  public double getHoodMotorRotations() {
    return hoodEncoder.getPosition();
  }

  /** @return Angle in Rad */
  public double getHoodAngle() {
    return 2 * Math.PI * (hoodEncoder.getPosition() / (HoodConstants.kMotorGearRatio * HoodConstants.kEncoderGearRatio));
  }

  /** @return Velocity in RPM */
  public double getHoodVelocity() {
    return hoodEncoder.getVelocity();
  }

  /** @return Current in Amps */
  public double getShooterPrimaryCurrent() {
    return shooterPrimaryMotor.getOutputCurrent();
  }

  /** @return Current in Amps */
  public double getShooterSecondaryCurrent() {
    return shooterPrimaryMotor.getOutputCurrent();
  }

  /** @return Current in Amps */
  public double getHoodPrimaryCurrent() {
    return hoodPrimaryMotor.getOutputCurrent();
  }

  /** @return Current in Amps */
  public double getHoodSecondaryCurrent() {
    return hoodSecondaryMotor.getOutputCurrent();
  }

  /** Publish continuous values to network table */
  public void updateNetworkTable() {
    shooterTable.getEntry(NetworkTableNames.Shooter.kVelocityRPM)
      .setNumber(getShooterVelocity());
    hoodTable.getEntry(NetworkTableNames.Hood.kVelocityRPM)
      .setNumber(getHoodVelocity());
    hoodTable.getEntry(NetworkTableNames.Hood.kPositionRotations)
      .setNumber(getHoodMotorRotations());
    hoodTable.getEntry(NetworkTableNames.Hood.kTargetRotations)
      .setNumber(rotations);

    shooterTable.getEntry(NetworkTableNames.Shooter.kPrimaryCurrent)
      .setNumber(getShooterVelocity());
    shooterTable.getEntry(NetworkTableNames.Shooter.kSecondaryCurrent)
      .setNumber(getShooterVelocity());
    hoodTable.getEntry(NetworkTableNames.Hood.kPrimaryCurrent)
      .setNumber(getHoodVelocity());
    hoodTable.getEntry(NetworkTableNames.Hood.kSecondaryCurrent)
      .setNumber(getHoodVelocity());
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    
    updateNetworkTable();

    setPIDAngle();
  }

  /** This method will be called once per scheduler run during simulation */
  @Override
  public void simulationPeriodic() {
    
  }
}