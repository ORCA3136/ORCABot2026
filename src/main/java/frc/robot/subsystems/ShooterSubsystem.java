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

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
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

  double shooterVelocityTarget = 0;  // Where we want to be (set by commands)
  double shooterVelocity = 0;        // Current ramped setpoint (fed to PID each cycle)
  boolean toggleDirection = false;
  double rotations; // Position of the Hood in Rotations
  boolean hoodMovingForward = true; // true is positive
  
  final SparkFlex shooterPrimaryMotor = new SparkFlex(CanIdConstants.kShooterPrimaryCanId, MotorType.kBrushless);
  final SparkFlex shooterSecondaryMotor = new SparkFlex(CanIdConstants.kShooterSecondaryCanId, MotorType.kBrushless);

  final SparkMax hoodPrimaryMotor = new SparkMax(CanIdConstants.kHoodPrimaryCanId, MotorType.kBrushless);
  final SparkMax hoodSecondaryMotor = new SparkMax(CanIdConstants.kHoodSecondaryCanId, MotorType.kBrushless);

  final RelativeEncoder shooterEncoder = shooterPrimaryMotor.getEncoder();

  final AbsoluteEncoder hoodEncoder = hoodPrimaryMotor.getAbsoluteEncoder();

  final SparkClosedLoopController shooterPrimaryPIDController = shooterPrimaryMotor.getClosedLoopController();
  final SparkClosedLoopController shooterSecondaryPIDController = shooterSecondaryMotor.getClosedLoopController();

  final SparkClosedLoopController hoodPIDController = hoodPrimaryMotor.getClosedLoopController();

  final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  final NetworkTable shooterTable = networkTable.getTable(NetworkTableNames.Shooter.kTable);
  final NetworkTable hoodTable = networkTable.getTable(NetworkTableNames.Hood.kTable);

  // Cached NetworkTable entries — avoids hash lookups every cycle (50Hz)
  private final NetworkTableEntry velocityEntryShooter = shooterTable.getEntry(NetworkTableNames.Shooter.kVelocityRPM);
  private final NetworkTableEntry targetEntryShooter = shooterTable.getEntry(NetworkTableNames.Shooter.kTargetRPM);
  private final NetworkTableEntry rampedSetpointEntryShooter = shooterTable.getEntry(NetworkTableNames.Shooter.kRampedSetpoint);
  private final NetworkTableEntry primaryCurrentEntryShooter = shooterTable.getEntry(NetworkTableNames.Shooter.kPrimaryCurrent);
  private final NetworkTableEntry secondaryCurrentEntryShooter = shooterTable.getEntry(NetworkTableNames.Shooter.kSecondaryCurrent);
  private final NetworkTableEntry readyEntryShooter = shooterTable.getEntry("Ready");
  
  // Same thing but for the hood
  private final NetworkTableEntry velocityEntryHood = hoodTable.getEntry(NetworkTableNames.Hood.kVelocityRPM);
  private final NetworkTableEntry positionEntryHood = hoodTable.getEntry(NetworkTableNames.Hood.kPositionRotations);
  private final NetworkTableEntry targetEntryHood = hoodTable.getEntry(NetworkTableNames.Hood.kTargetRotations);
  private final NetworkTableEntry angleEntryHood = hoodTable.getEntry(NetworkTableNames.Hood.kAngleDegrees);
  private final NetworkTableEntry primaryCurrentEntryHood = hoodTable.getEntry(NetworkTableNames.Hood.kPrimaryCurrent);
  private final NetworkTableEntry secondaryCurrentEntryHood = hoodTable.getEntry(NetworkTableNames.Hood.kSecondaryCurrent);


  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    shooterPrimaryMotor.configure(ShooterConfigs.primaryShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterSecondaryMotor.configure(ShooterConfigs.secondaryShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    hoodPrimaryMotor.configure(HoodConfigs.primaryHoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hoodSecondaryMotor.configure(HoodConfigs.secondaryHoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // updateHoodTarget(0);
  }

  public double calculateShooterFeedForward() {
    // FF pivot = Ksta + Kvel * TarVel + Kgrav * cos(angle) + Kaccel * RobAccel * sin(angle)
    double ff = ShooterConstants.kS + shooterVelocity * ShooterConstants.kVelocityModifier;
    return shooterVelocity < 100 ? 0 : ff;
  }

  /** Calculates the current hood feedforward
   * {@summary Hood feedforward includes gravitational force, static loss, air resistance, and robot acceleration} */
  public double calculateHodFeedForward() {
    // FF pivot = Ksta + Kvel * TarVel + Kgrav * cos(angle) + Kaccel * RobAccel * sin(angle)
    return HoodConstants.kS + HoodConstants.kG * Math.cos(getHoodAngle());
  }

  /**
   * Sets the shooters velocity
   * 
   * use "setShooterVelocityTarget" to change shooterVelocity variable in this subsystem
   */
  private void setShooterPIDVelocity() {
    // Calculate FF once and reuse — it uses the ramped setpoint (not the final target)
    // so the FF matches what the PID is currently tracking.
    double ff = calculateShooterFeedForward();
    shooterPrimaryPIDController.setSetpoint(shooterVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ff);
    shooterSecondaryPIDController.setSetpoint(shooterVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ff);
  }

  /** Sets the hood setpoint angle */
  public void setHoodPIDAngle() {
    hoodPIDController.setSetpoint(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0, calculateHodFeedForward());
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


    if (shooterVelocityTarget > ShooterConstants.kVelocityMax) shooterVelocityTarget = ShooterConstants.kVelocityMax;
    if (shooterVelocityTarget < 0) shooterVelocityTarget = 0;
  }

  /** Decreases the speed of the shooter by ~ 100 RPM */
  public void decreaseShooterVelocity() {
    shooterVelocityTarget -= 100;
    if (shooterVelocityTarget < 0) shooterVelocityTarget = 0;
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

  /** @return Primary motor for simulation access */
  public SparkFlex getShooterPrimaryMotor() {
    return shooterPrimaryMotor;
  }

  /** @return true if the shooter ramp has finished and flywheel is within tolerance of target RPM */
  public boolean isShooterReady() {
    return shooterVelocityTarget > 0
        && shooterVelocity >= shooterVelocityTarget
        && Math.abs(getShooterVelocity() - shooterVelocityTarget) < ShooterConstants.kReadyToleranceRPM;
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
  public SparkMax getPrimaryHoodMotor() {
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
    velocityEntryShooter.setDouble(getShooterVelocity());
    targetEntryShooter.setDouble(getShooterTarget());
    rampedSetpointEntryShooter.setDouble(getRampedSetpoint());
    primaryCurrentEntryShooter.setDouble(getShooterPrimaryCurrent());
    secondaryCurrentEntryShooter.setDouble(getShooterSecondaryCurrent());
    readyEntryShooter.setBoolean(isShooterReady());

    velocityEntryHood.setDouble(getHoodVelocity());
    positionEntryHood.setDouble(getHoodMotorRotations());
    targetEntryHood.setDouble(rotations);
    angleEntryHood.setDouble(Math.toDegrees(getHoodAngle()));
    primaryCurrentEntryHood.setDouble(getHoodPrimaryCurrent());
    secondaryCurrentEntryHood.setDouble(getHoodSecondaryCurrent());
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
    setHoodPIDAngle();
  }

  /** This method will be called once per scheduler run during simulation */
  @Override
  public void simulationPeriodic() {
    
  }
}