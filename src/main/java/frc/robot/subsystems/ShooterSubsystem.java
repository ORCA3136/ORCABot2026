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

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ShooterConfigs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.NetworkTableNames;


/*
 * 
 * This subsystem is for all Shooter operations
 * Including:
 *    logging shooter data
 *    Managing the speeds of the flywheels
 * 
 */


public class ShooterSubsystem extends SubsystemBase {

  SwerveSubsystem m_swerveSubsystem;

  double shooterVelocityTarget = 0;  // Where we want to be (set by commands)
  double shooterVelocity = 0;        // Current ramped setpoint (fed to PID each cycle)
  boolean toggleDirection = false;
  double hoodTarget; // Position of the Hood in Rotations
  boolean hoodMovingForward = true; // true is positive
  
  final SparkFlex shooterPrimaryMotor = new SparkFlex(CanIdConstants.kShooterPrimaryCanId, MotorType.kBrushless);
  final SparkFlex shooterSecondaryMotor = new SparkFlex(CanIdConstants.kShooterSecondaryCanId, MotorType.kBrushless);

  final RelativeEncoder shooterEncoder = shooterPrimaryMotor.getEncoder();

  final SparkClosedLoopController shooterPrimaryPIDController = shooterPrimaryMotor.getClosedLoopController();
  final SparkClosedLoopController shooterSecondaryPIDController = shooterSecondaryMotor.getClosedLoopController();

  final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  final NetworkTable shooterTable = networkTable.getTable(NetworkTableNames.Shooter.kTable);

  // Cached NetworkTable entries — avoids hash lookups every cycle (50Hz)
  private final NetworkTableEntry velocityEntryShooter = shooterTable.getEntry(NetworkTableNames.Shooter.kVelocityRPM);
  private final NetworkTableEntry targetEntryShooter = shooterTable.getEntry(NetworkTableNames.Shooter.kTargetRPM);
  private final NetworkTableEntry rampedSetpointEntryShooter = shooterTable.getEntry(NetworkTableNames.Shooter.kRampedSetpoint);
  private final NetworkTableEntry primaryCurrentEntryShooter = shooterTable.getEntry(NetworkTableNames.Shooter.kPrimaryCurrent);
  private final NetworkTableEntry secondaryCurrentEntryShooter = shooterTable.getEntry(NetworkTableNames.Shooter.kSecondaryCurrent);
  private final NetworkTableEntry readyEntryShooter = shooterTable.getEntry("Ready");

  private InterpolatingDoubleTreeMap shooterSpeedOnlyMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap fuelAirTimeMap = new InterpolatingDoubleTreeMap();

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(SwerveSubsystem swerveSubsystem) {

    

    m_swerveSubsystem = swerveSubsystem;

    shooterPrimaryMotor.configure(ShooterConfigs.primaryShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterSecondaryMotor.configure(ShooterConfigs.secondaryShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    addMapValues();
  }

  private void addMapValues() {
    
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(60 )), Double.valueOf(1905)); // Value is in RPM
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(70 )), Double.valueOf(1940));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(80 )), Double.valueOf(2005));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(90 )), Double.valueOf(2100));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(100)), Double.valueOf(2150));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(110)), Double.valueOf(2205));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(120)), Double.valueOf(2260));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(130)), Double.valueOf(2305));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(140)), Double.valueOf(2370));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(150)), Double.valueOf(2445));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(160)), Double.valueOf(2500));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(170)), Double.valueOf(2570));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(180)), Double.valueOf(2625));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(200)), Double.valueOf(2785));

    fuelAirTimeMap.put(Double.valueOf(Units.inchesToMeters(70 )), Double.valueOf(0.0029)); // Value is in seconds
    fuelAirTimeMap.put(Double.valueOf(Units.inchesToMeters(90 )), Double.valueOf(0.00365));
    fuelAirTimeMap.put(Double.valueOf(Units.inchesToMeters(120)), Double.valueOf(0.0044));
    fuelAirTimeMap.put(Double.valueOf(Units.inchesToMeters(150)), Double.valueOf(0.00485));
  }

  public double calculateShooterFeedForward() {
    // FF pivot = Ksta + Kvel * TarVel + Kgrav * cos(angle) + Kaccel * RobAccel * sin(angle)
    double ff = ShooterConstants.kS + shooterVelocity * ShooterConstants.kVelocityModifier;
    return ff;
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
    if (Math.abs(shooterVelocity) > 200) {
      shooterPrimaryPIDController.setSetpoint(shooterVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ff);
      shooterSecondaryPIDController.setSetpoint(shooterVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ff);
    } else {
      shooterPrimaryMotor.set(0);
      shooterSecondaryMotor.set(0);
    }

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

  /** Increases the speed of the shooter by ~ 100 RPM */
  public void increaseShooterVelocity(int level) {
    switch (level) {
      case 1:
        shooterVelocityTarget += 5;
        break;
      case 2:
        shooterVelocityTarget += 50;
        break;
      case 3:
        shooterVelocityTarget += 100;
        break;
      case 4:
        shooterVelocityTarget += 500;
        break;
      case -1:
        shooterVelocityTarget -= 5;
        break;
      case -2:
        shooterVelocityTarget -= 50;
        break;
      case -3:
        shooterVelocityTarget -= 100;
        break;
      case -4:
        shooterVelocityTarget -= 500;
        break;
      default:
        break;
    }

    if (shooterVelocityTarget > ShooterConstants.kVelocityMax) 
        shooterVelocityTarget = ShooterConstants.kVelocityMax;
    else if (shooterVelocityTarget < 0) 
        shooterVelocityTarget = 0;
  }

  /** Decreases the speed of the shooter by ~ 100 RPM */
  public void decreaseShooterVelocity() {
    shooterVelocityTarget -= 100;
    if (shooterVelocityTarget < 0) shooterVelocityTarget = 0;
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

  public void setShooterMapOnly() {
    double distanceToHub = m_swerveSubsystem.getDistanceToHubMeters();
    // set shooter based on distance
    shooterVelocityTarget = shooterSpeedOnlyMap.get(distanceToHub);
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

  /** Ramp the setpoint toward the target each cycle. */
  private void rampSetpoint() {
    if (shooterVelocity < shooterVelocityTarget) {
      shooterVelocity = Math.min(shooterVelocity + ShooterConstants.kRampUpRate, shooterVelocityTarget);
    } else if (shooterVelocity > shooterVelocityTarget) {
      shooterVelocity = Math.max(shooterVelocity - ShooterConstants.kRampDownRate, shooterVelocityTarget);
    }
  }


  /** Publish continuous values to network table */
  public void updateNetworkTable() {
    velocityEntryShooter.setDouble(getShooterVelocity());
    targetEntryShooter.setDouble(getShooterTarget());
    rampedSetpointEntryShooter.setDouble(getRampedSetpoint());
    primaryCurrentEntryShooter.setDouble(getShooterPrimaryCurrent());
    secondaryCurrentEntryShooter.setDouble(getShooterSecondaryCurrent());
    readyEntryShooter.setBoolean(isShooterReady());
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