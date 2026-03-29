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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ShooterConfigs;
import frc.robot.Constants.FieldPositions;
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
  private final NetworkTableEntry leadCompXEntry = shooterTable.getEntry(NetworkTableNames.Shooter.kLeadCompX);
  private final NetworkTableEntry leadCompYEntry = shooterTable.getEntry(NetworkTableNames.Shooter.kLeadCompY);
  private final NetworkTableEntry leadCompDistEntry = shooterTable.getEntry(NetworkTableNames.Shooter.kLeadCompDistance);
  private final NetworkTableEntry airTimeEntry = shooterTable.getEntry(NetworkTableNames.Shooter.kAirTimeSec);
  private final NetworkTableEntry actualDistanceEntry = shooterTable.getEntry(NetworkTableNames.Shooter.kActualDistanceM);
  private final NetworkTableEntry fieldVelXEntry = shooterTable.getEntry(NetworkTableNames.Shooter.kFieldVelX);
  private final NetworkTableEntry fieldVelYEntry = shooterTable.getEntry(NetworkTableNames.Shooter.kFieldVelY);
  private final NetworkTableEntry leadCompGainEntry = shooterTable.getEntry(NetworkTableNames.Shooter.kLeadCompGain);

  // Last-computed lead compensation values for NT telemetry
  private double lastLeadCompX = 0;
  private double lastLeadCompY = 0;
  private double lastLeadCompDistance = 0;
  private double lastAirTimeSec = 0;
  private Translation2d lastLeadCompHubTranslation = new Translation2d();

  // Distance clamp range for lead-compensated shooting (meters)
  private static final double kMinShootDistanceM = Units.inchesToMeters(70);
  private static final double kMaxShootDistanceM = Units.inchesToMeters(170);

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
    
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(70 )), Double.valueOf(2515));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(80 )), Double.valueOf(2575));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(90 )), Double.valueOf(2625));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(100)), Double.valueOf(2700));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(110)), Double.valueOf(2750));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(120)), Double.valueOf(2825));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(130)), Double.valueOf(2910));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(140)), Double.valueOf(3000));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(150)), Double.valueOf(3125));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(160)), Double.valueOf(3250));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(170)), Double.valueOf(3350));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(180)), Double.valueOf(3500));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(200)), Double.valueOf(3875));

    fuelAirTimeMap.put(Double.valueOf(Units.inchesToMeters(70 )), Double.valueOf(0.78)); // Value is in seconds
    fuelAirTimeMap.put(Double.valueOf(Units.inchesToMeters(80 )), Double.valueOf(0.79));
    fuelAirTimeMap.put(Double.valueOf(Units.inchesToMeters(90 )), Double.valueOf(0.83));
    fuelAirTimeMap.put(Double.valueOf(Units.inchesToMeters(100 )), Double.valueOf(0.9));
    fuelAirTimeMap.put(Double.valueOf(Units.inchesToMeters(110)), Double.valueOf(1.0));
    fuelAirTimeMap.put(Double.valueOf(Units.inchesToMeters(120)), Double.valueOf(1.02));
    fuelAirTimeMap.put(Double.valueOf(Units.inchesToMeters(130)), Double.valueOf(1.08));
    fuelAirTimeMap.put(Double.valueOf(Units.inchesToMeters(140)), Double.valueOf(1.1));
    fuelAirTimeMap.put(Double.valueOf(Units.inchesToMeters(160)), Double.valueOf(1.14));
    fuelAirTimeMap.put(Double.valueOf(Units.inchesToMeters(180)), Double.valueOf(1.18));
    fuelAirTimeMap.put(Double.valueOf(Units.inchesToMeters(200)), Double.valueOf(1.3));
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

  public void setShooterMapOnly(double distance) {
    double distanceToHub = Units.inchesToMeters(distance);
    // set shooter based on distance
    shooterVelocityTarget = 2825; // shooterSpeedOnlyMap.get(distanceToHub);
  }

  /**
   * Sets shooter RPM using the lead-compensated distance to the hub.
   * Accounts for robot motion so RPM matches the effective shot distance.
   */
  public void setShooterMapLeadCompensated() {
    Translation2d hubPos = m_swerveSubsystem.getAlliance() == DriverStation.Alliance.Red
        ? FieldPositions.kRedFieldElements.get(0)
        : FieldPositions.kBlueFieldElements.get(0);
    Translation2d compensatedHub = getLeadCompensatedHubPosition(hubPos);

    // RPM: use ACTUAL distance to hub (not compensated) — RPM map was tuned stationary
    double actualDistance = MathUtil.clamp(
        m_swerveSubsystem.getDistanceToHubMeters(), kMinShootDistanceM, kMaxShootDistanceM);
    shooterVelocityTarget = shooterSpeedOnlyMap.get(actualDistance);

    // Cache compensated position for the aim stream (heading only)
    lastLeadCompHubTranslation = compensatedHub;
    Translation2d offset = compensatedHub.minus(hubPos);
    lastLeadCompX = offset.getX();
    lastLeadCompY = offset.getY();
    lastLeadCompDistance = actualDistance;
  }

  /**
   * Returns the last-computed lead-compensated hub position.
   * Used by RobotContainer to continuously update the aim target.
   */
  public Translation2d getLeadCompensatedHubTranslation() {
    return lastLeadCompHubTranslation;
  }

  /**
   * Returns a lead-compensated aim point for shoot-on-the-move.
   * Offsets the hub position by (robot_velocity * fuel_air_time) so the
   * heading lock aims where the hub "will be" relative to the moving ball.
   */
  public Translation2d getLeadCompensatedHubPosition(Translation2d hubPos) {
    double distance = m_swerveSubsystem.getDistanceToHubMeters();
    double airTime = fuelAirTimeMap.get(distance);
    lastAirTimeSec = airTime;
    ChassisSpeeds fieldVel = m_swerveSubsystem.getFieldVelocity();
    double gain = ShooterConstants.kLeadCompGain;
    // Offset aim point opposite to robot velocity (ball inherits robot momentum)
    return hubPos.minus(new Translation2d(
        fieldVel.vxMetersPerSecond * airTime * gain,
        fieldVel.vyMetersPerSecond * airTime * gain
    ));
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
    leadCompXEntry.setDouble(lastLeadCompX);
    leadCompYEntry.setDouble(lastLeadCompY);
    leadCompDistEntry.setDouble(lastLeadCompDistance);
    airTimeEntry.setDouble(lastAirTimeSec);
    actualDistanceEntry.setDouble(m_swerveSubsystem.getDistanceToHubMeters());
    ChassisSpeeds fieldVel = m_swerveSubsystem.getFieldVelocity();
    fieldVelXEntry.setDouble(fieldVel.vxMetersPerSecond);
    fieldVelYEntry.setDouble(fieldVel.vyMetersPerSecond);
    leadCompGainEntry.setDouble(ShooterConstants.kLeadCompGain);
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