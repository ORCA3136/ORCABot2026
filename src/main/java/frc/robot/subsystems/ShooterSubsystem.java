// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import frc.robot.Constants.HoodConstants;


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
  
  final SparkFlex shooterPrimaryMotor = new SparkFlex(CanIdConstants.kShooterPrimaryCanId, MotorType.kBrushless);
  final SparkFlex shooterSecondaryMotor = new SparkFlex(CanIdConstants.kShooterSecondaryCanId, MotorType.kBrushless);

  final SparkMax hoodPrimaryMotor = new SparkMax(CanIdConstants.kHoodPrimaryCanId, MotorType.kBrushless);
  final SparkMax hoodSecondaryMotor = new SparkMax(CanIdConstants.kHoodSecondaryCanId, MotorType.kBrushless);

  final RelativeEncoder shooterEncoder = shooterPrimaryMotor.getEncoder();
  final RelativeEncoder hoodEncoder = hoodPrimaryMotor.getEncoder();

  final SparkClosedLoopController hoodPIDController = hoodPrimaryMotor.getClosedLoopController();

  final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  final NetworkTable shooterTable = networkTable.getTable(NetworkTableNames.Shooter.kShooter);
  final NetworkTable hoodTable = networkTable.getTable(NetworkTableNames.Hood.kHood);

  public ShooterSubsystem() {

    shooterPrimaryMotor.configure(ShooterConfigs.primaryShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterSecondaryMotor.configure(ShooterConfigs.secondaryShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public double calculateFeedForward() {
    return HoodConstants.kG * Math.cos(Units.degreesToRadians(getHoodPosition()));
  }

  /**
   * @param Angle is in Degrees
   */
  public void setPIDAngle(double angle) {
    double rotations = angle / 360;
    hoodPIDController.setSetpoint(rotations, ControlType.kMAXMotionPositionControl,ClosedLoopSlot.kSlot0,0);
  }

  /**
   * In RPM
   * Increases the power of the shooter by about 5% (caps at 6500)
   */
  public void increaseShooterVelocity() {
    shooterVelocity += 350;
    if (shooterVelocity > 6500) shooterVelocity = 6500;

    setShooterVelocity(shooterVelocity);
  }

  /**
   * In RPM
   * Decreases the power of the shooter by about 5% (Defaults to 0 if you try to go negative)
   */
  public void decreaseShooterVelocity() {
    shooterVelocity -= 350;
    if (shooterVelocity < 0) shooterVelocity = 0;
      
    setShooterVelocity(shooterVelocity);
  }
  
  /**
   * @param Velocity is in RPM
   */
  public void setShooterVelocityHigh() {
    shooterPrimaryMotor.set(ShooterConstants.kVelocityHigh);
  }

  /**
   * @param Velocity is in RPM
   */
  public void setShooterVelocityNone() {
    shooterPrimaryMotor.set(0);
  }

  /**
   * @param velocity is in RPM
   */
  public void setShooterVelocity(double velocity) {
    shooterPrimaryMotor.set(velocity / 6500);
    shooterSecondaryMotor.set(velocity / 6500);
  }

  /**
   * @return Velocity in RPM
   */
  public double getShooterVelocity() {
    return shooterEncoder.getVelocity();
  }

  /**
   * @param Velocity is in RPM
   */
  public void setHoodVelocity(double velocity) {
    hoodPrimaryMotor.set(velocity / 6500);
    hoodSecondaryMotor.set(velocity / 6500);
  }

  /**
   * @return Velocity in RPM
   */
  public double getHoodVelocity() {
    return hoodEncoder.getVelocity();
  }

  /**
   * @return Velocity in RPM
   */
  public double getHoodPosition() {
    return hoodEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterTable.getEntry(NetworkTableNames.Shooter.kVelocityRPM)
      .setNumber(getShooterVelocity());
    hoodTable.getEntry(NetworkTableNames.Hood.kVelocityRPM)
      .setNumber(getHoodVelocity());
    hoodTable.getEntry(NetworkTableNames.Hood.kPositionRotations)
      .setNumber(getHoodPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}