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

  double shooterVelocityTarget = 0;  // Where we want to be (set by commands)
  double shooterVelocity = 0;        // Current ramped setpoint (fed to PID each cycle)
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
    shooterVelocityTarget = target;
  }

  /** @return The final target velocity (before ramping) */
  public double getShooterTarget() {
    return shooterVelocityTarget;
  }

  /** @return The current ramped setpoint being fed to PID */
  public double getRampedSetpoint() {
    return shooterVelocity;
  }

  public void setToggleDirection(boolean toggle) {
    toggleDirection = toggle;
  };

  /** Increases the speed of the shooter by ~ 100 RPM */
  public void increaseShooterVelocity(int level) {
    if (level == 1) {
      if (!toggleDirection) shooterVelocityTarget += 5;
      else shooterVelocityTarget -= 5;
    }

    if (level == 2) {
      if (!toggleDirection) shooterVelocityTarget += 50;
      else shooterVelocityTarget -= 50;
    }

    if (level == 3) {
      if (!toggleDirection) shooterVelocityTarget += 100;
      else shooterVelocityTarget -= 100;
    }

    if (level == 4) {
      if (!toggleDirection) shooterVelocityTarget += 500;
      else shooterVelocityTarget -= 500;
    }


    if (shooterVelocityTarget > 6500) shooterVelocityTarget = 6500;
    if (shooterVelocityTarget < 0) shooterVelocityTarget = 0;
  }

  /** Decreases the speed of the shooter by ~ 100 RPM */
  public void decreaseShooterVelocity() {
    shooterVelocityTarget -= 100;
    if (shooterVelocityTarget < 0) shooterVelocityTarget = 0;
  }

  /** @return Primary motor for simulation access */
  public SparkFlex getPrimaryMotor() {
    return shooterPrimaryMotor;
  }

  /** @return true if the shooter is spinning and within tolerance of its target RPM */
  public boolean isShooterReady() {
    return shooterVelocityTarget > 0
        && Math.abs(getShooterVelocity() - shooterVelocityTarget) < FuelPathConstants.kShooterReadyToleranceRPM;
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

    shooterTable.getEntry(NetworkTableNames.Shooter.kRampedSetpoint)
      .setNumber(getRampedSetpoint());
    shooterTable.getEntry(NetworkTableNames.Shooter.kPrimaryCurrent)
      .setNumber(getShooterPrimaryCurrent());
    shooterTable.getEntry(NetworkTableNames.Shooter.kSecondaryCurrent)
      .setNumber(getShooterSecondaryCurrent());

  }

  /** Ramp the setpoint toward the target each cycle. */
  private void rampSetpoint() {
    if (shooterVelocity < shooterVelocityTarget) {
      shooterVelocity = Math.min(shooterVelocity + ShooterConstants.kRampUpRate, shooterVelocityTarget);
    } else if (shooterVelocity > shooterVelocityTarget) {
      shooterVelocity = Math.max(shooterVelocity - ShooterConstants.kRampDownRate, shooterVelocityTarget);
    }
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    rampSetpoint();
    updateNetworkTable();
    setShooterPIDVelocity();
  }

  /** This method will be called once per scheduler run during simulation */
  @Override
  public void simulationPeriodic() {
    
  }
}