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
import edu.wpi.first.units.measure.Velocity;
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
  boolean toggleDirection = false;
  
  final SparkFlex shooterPrimaryMotor = new SparkFlex(CanIdConstants.kShooterPrimaryCanId, MotorType.kBrushless);
  final SparkFlex shooterSecondaryMotor = new SparkFlex(CanIdConstants.kShooterSecondaryCanId, MotorType.kBrushless);

  final RelativeEncoder shooterEncoder = shooterPrimaryMotor.getEncoder();

  final SparkClosedLoopController shooterPrimaryPIDController = shooterPrimaryMotor.getClosedLoopController();
  final SparkClosedLoopController shooterSecondaryPIDController = shooterSecondaryMotor.getClosedLoopController();

  final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  final NetworkTable shooterTable = networkTable.getTable(NetworkTableNames.Shooter.kTable);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    shooterPrimaryMotor.configure(ShooterConfigs.primaryShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterSecondaryMotor.configure(ShooterConfigs.secondaryShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public double calculateFeedForward() {
    // FF pivot = Ksta + Kvel * TarVel + Kgrav * cos(angle) + Kaccel * RobAccel * sin(angle)
    double ff = ShooterConstants.kS + shooterVelocity * ShooterConstants.kVelocityModifier;
    return shooterVelocity == 0 ? 0 : ff;
  }

  /**
   * Sets the shooters velocity
   * 
   * use "setShooterVelocityTarget" to change shooterVelocity variable in this subsystem
   */
  private void setShooterPIDVelocity() {
    shooterPrimaryPIDController.setSetpoint(shooterVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, calculateFeedForward());
    shooterSecondaryPIDController.setSetpoint(shooterVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, calculateFeedForward());
  }

  public void setShooterVelocityTarget(double target) {
    shooterVelocity = target;
  }

  /**  */
  public double getShooterTarget() {
    return shooterVelocity;
  }

  public void setToggleDirection(boolean toggle) {
    toggleDirection = toggle;
  };

  /** Increases the speed of the shooter by ~ 100 RPM */
  public void increaseShooterVelocity(int level) {
    if (level == 1) {
      if (!toggleDirection) shooterVelocity += 5;
      else shooterVelocity -= 5;
    }

    if (level == 2) {
      if (!toggleDirection) shooterVelocity += 50;
      else shooterVelocity -= 50;
    }

    if (level == 3) {
      if (!toggleDirection) shooterVelocity += 100;
      else shooterVelocity -= 100;
    }

    if (level == 4) {
      if (!toggleDirection) shooterVelocity += 500;
      else shooterVelocity -= 500;
    }

    
    if (shooterVelocity > 6500) shooterVelocity = 6500;
    if (shooterVelocity < 0) shooterVelocity = 0;
  }

  /** Decreases the speed of the shooter by ~ 100 RPM */ 
  public void decreaseShooterVelocity() {
    shooterVelocity -= 100;
    if (shooterVelocity < 0) shooterVelocity = 0;
  }

  /** @return Velocity in RPM */
  public double getShooterVelocity() {
    return shooterEncoder.getVelocity();
  }

  /** @return Current in Amps */
  public double getShooterPrimaryCurrent() {
    return shooterPrimaryMotor.getOutputCurrent();
  }

  /** @return Current in Amps */
  public double getShooterSecondaryCurrent() {
    return shooterSecondaryMotor.getOutputCurrent();
  }

  /** Publish continuous values to network table */
  public void updateNetworkTable() {
    shooterTable.getEntry(NetworkTableNames.Shooter.kVelocityRPM)
      .setNumber(getShooterVelocity());
    shooterTable.getEntry(NetworkTableNames.Shooter.kTargetRPM)
      .setNumber(getShooterTarget());

    shooterTable.getEntry(NetworkTableNames.Shooter.kPrimaryCurrent)
      .setNumber(getShooterPrimaryCurrent());
    shooterTable.getEntry(NetworkTableNames.Shooter.kSecondaryCurrent)
      .setNumber(getShooterSecondaryCurrent());

  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    
    updateNetworkTable();

    setShooterPIDVelocity();
  }

  /** This method will be called once per scheduler run during simulation */
  @Override
  public void simulationPeriodic() {
    
  }
}