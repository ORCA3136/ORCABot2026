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
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
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

  SwerveSubsystem m_swerveSubsystem;

  double shooterVelocityTarget = 0;  // Where we want to be (set by commands)
  double shooterVelocity = 0;        // Current ramped setpoint (fed to PID each cycle)
  boolean toggleDirection = false;
  double hoodTarget; // Position of the Hood in Rotations
  boolean hoodMovingForward = true; // true is positive

  private double hoodCalibrationOffset = 0.0;
  private DigitalInput hoodLimitSwitch = null;
  private boolean prevLimitSwitchPressed = false;
  private boolean hoodManualOverride = false;
  private boolean hoodEncoderValid = true;

  // Stall detection for auto re-zero
  private static final String kCalibrationOffsetKey = "HoodCalibrationOffset";
  private static final double kStallCurrentThreshold = 12.0; // Amps
  private static final double kStallVelocityThreshold = 0.5; // RPM
  private static final double kStallDebounceSec = 0.5;
  private final Debouncer stallDebouncer = new Debouncer(kStallDebounceSec, DebounceType.kRising);
  
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
  private final NetworkTableEntry calibrationOffsetEntryHood = hoodTable.getEntry("Calibration Offset");
  private final NetworkTableEntry encoderValidEntryHood = hoodTable.getEntry("Encoder Valid");
  private final NetworkTableEntry rawPositionEntryHood = hoodTable.getEntry("Raw Position");


  private InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap shooterSpeedOnlyMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

  private static final double additionalRPM = 65;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(SwerveSubsystem swerveSubsystem) {

    

    m_swerveSubsystem = swerveSubsystem;

    shooterPrimaryMotor.configure(ShooterConfigs.primaryShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterSecondaryMotor.configure(ShooterConfigs.secondaryShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    hoodPrimaryMotor.configure(HoodConfigs.primaryHoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    hoodSecondaryMotor.configure(HoodConfigs.secondaryHoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    hoodCalibrationOffset = Preferences.getDouble(kCalibrationOffsetKey, 0.0);

    setHoodTarget(HoodConstants.kEncoderOffset);

    addMapValues();
  }

  private void clampHoodTarget() {
    if (hoodTarget < HoodConstants.kMinEncoderPosition) {
      hoodTarget = HoodConstants.kMinEncoderPosition;
    } else if (hoodTarget > HoodConstants.kMaxEncoderPosition) {
      hoodTarget = HoodConstants.kMaxEncoderPosition;
    }
  }

  private void addMapValues() {
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters(  8)), Double.valueOf(1615 + additionalRPM));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters( 20)), Double.valueOf(1685 + additionalRPM));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters( 32)), Double.valueOf(1755 + additionalRPM));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters( 44)), Double.valueOf(1825 + additionalRPM));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters( 56)), Double.valueOf(1885 + additionalRPM));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters( 68)), Double.valueOf(1950 + additionalRPM));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters( 80)), Double.valueOf(2035 + additionalRPM));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters( 92)), Double.valueOf(2105 + additionalRPM));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters(104)), Double.valueOf(2125 + additionalRPM));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters(116)), Double.valueOf(2150 + additionalRPM));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters(128)), Double.valueOf(2200 + additionalRPM));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters(140)), Double.valueOf(2275 + additionalRPM));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters(152)), Double.valueOf(2350 + additionalRPM));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters(164)), Double.valueOf(2425 + additionalRPM));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters(176)), Double.valueOf(2500 + additionalRPM));

    // shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters(116)), Double.valueOf(2250));
    // shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters(134)), Double.valueOf(2300));
    shooterSpeedMap.put(Double.valueOf(Units.inchesToMeters(200)), Double.valueOf(2850 + additionalRPM)); // 2800

    hoodAngleMap.put(Double.valueOf(Units.inchesToMeters(  8)), Double.valueOf(0.47));
    hoodAngleMap.put(Double.valueOf(Units.inchesToMeters( 20)), Double.valueOf(0.52));
    hoodAngleMap.put(Double.valueOf(Units.inchesToMeters( 32)), Double.valueOf(0.54));
    hoodAngleMap.put(Double.valueOf(Units.inchesToMeters( 44)), Double.valueOf(0.58));
    hoodAngleMap.put(Double.valueOf(Units.inchesToMeters( 56)), Double.valueOf(0.61));
    hoodAngleMap.put(Double.valueOf(Units.inchesToMeters( 68)), Double.valueOf(0.65));
    hoodAngleMap.put(Double.valueOf(Units.inchesToMeters( 80)), Double.valueOf(1.11));
    hoodAngleMap.put(Double.valueOf(Units.inchesToMeters( 92)), Double.valueOf(1.24));
    hoodAngleMap.put(Double.valueOf(Units.inchesToMeters(104)), Double.valueOf(1.34));
    hoodAngleMap.put(Double.valueOf(Units.inchesToMeters(116)), Double.valueOf(1.38));
    hoodAngleMap.put(Double.valueOf(Units.inchesToMeters(128)), Double.valueOf(1.42));
    hoodAngleMap.put(Double.valueOf(Units.inchesToMeters(140)), Double.valueOf(1.50));
    hoodAngleMap.put(Double.valueOf(Units.inchesToMeters(152)), Double.valueOf(1.63));
    hoodAngleMap.put(Double.valueOf(Units.inchesToMeters(164)), Double.valueOf(1.70));
    hoodAngleMap.put(Double.valueOf(Units.inchesToMeters(176)), Double.valueOf(1.78));

    // hoodAngleMap.put(Double.valueOf(Units.inchesToMeters(116)), Double.valueOf(1.46));
    // hoodAngleMap.put(Double.valueOf(Units.inchesToMeters(134)), Double.valueOf(1.48));
    hoodAngleMap.put(Double.valueOf(Units.inchesToMeters(200)), Double.valueOf(1.82)); // 1.78




    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters( 60)), Double.valueOf(1950));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters( 90)), Double.valueOf(2100));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(116)), Double.valueOf(2300));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(116)), Double.valueOf(2300));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(159)), Double.valueOf(2550));
    shooterSpeedOnlyMap.put(Double.valueOf(Units.inchesToMeters(233)), Double.valueOf(3450));
  }

  public double calculateShooterFeedForward() {
    // FF pivot = Ksta + Kvel * TarVel + Kgrav * cos(angle) + Kaccel * RobAccel * sin(angle)
    double ff = ShooterConstants.kS + shooterVelocity * ShooterConstants.kVelocityModifier;
    return ff;
  }

  /** Calculates the current hood feedforward
   * {@summary Hood feedforward includes gravitational force, static loss, air resistance, and robot acceleration} */
  public double calculateHodFeedForward() {
    // FF pivot = Ksta + Kvel * TarVel + Kgrav * cos(angle) + Kaccel * RobAccel * sin(angle)
    return HoodConstants.kS + HoodConstants.kG * Math.cos(getHoodAngle());
  }

  /** Re-zero the hood encoder by computing a calibration offset from the current raw position.
   *  Call this when the hood is physically at the home (down) position. */
  public void reZeroHood() {
    double newOffset = HoodConstants.kEncoderOffset - hoodEncoder.getPosition();
    hoodTarget = HoodConstants.kEncoderOffset;
    clampHoodTarget();
    // Only persist if offset actually changed (avoid unnecessary flash writes)
    if (Math.abs(newOffset - hoodCalibrationOffset) > 0.01) {
      hoodCalibrationOffset = newOffset;
      Preferences.setDouble(kCalibrationOffsetKey, hoodCalibrationOffset);
    }
    // final SparkMaxConfig primaryHoodConfig = new SparkMaxConfig();
    // primaryHoodConfig.absoluteEncoder.zeroOffset(0.3 / 32);
    
    // hoodPrimaryMotor.configure(primaryHoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Enable a limit switch on the given DIO port for auto-re-zeroing the hood. */
  public void enableHoodLimitSwitch(int dioPort) {
    hoodLimitSwitch = new DigitalInput(dioPort);
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

  /** Sets the hood setpoint angle (compensates for calibration offset in raw encoder space) */
  public void setHoodPIDAngle() {
    if (hoodManualOverride) return;

    double correctedPosition = hoodEncoder.getPosition() + hoodCalibrationOffset;
    double margin = 0.5;
    if (correctedPosition < (HoodConstants.kMinEncoderPosition - margin)
        || correctedPosition > (HoodConstants.kMaxEncoderPosition + margin)) {
      // Encoder is reading garbage — stop motor rather than chasing nonsensical position
      hoodEncoderValid = false;
      hoodPrimaryMotor.set(0);
      return;
    }
    hoodEncoderValid = true;
    double rawSetpoint = hoodTarget - hoodCalibrationOffset;
    hoodPIDController.setSetpoint(rawSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, calculateHodFeedForward());
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

  public void setToggleDirection() {
    toggleDirection = !toggleDirection;
  };

  /** Increases the speed of the shooter by ~ 100 RPM */
  public void increaseShooterVelocity(int level) {
    if (level == 1) {
      shooterVelocityTarget += 5;
    }

    if (level == 2) {
      shooterVelocityTarget += 50;
    }

    if (level == 3) {
      shooterVelocityTarget += 100;
    }

    if (level == 4) {
      shooterVelocityTarget += 500;
    }

    if (level == -1) {
      shooterVelocityTarget -= 5;
    }

    if (level == -2) {
      shooterVelocityTarget -= 50;
    }

    if (level == -3) {
      shooterVelocityTarget -= 100;
    }

    if (level == -4) {
      shooterVelocityTarget -= 500;
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
    hoodTarget = HoodConstants.kEncoderOffset + (angle / 360) * HoodConstants.kEncoderGearRatio * HoodConstants.kMotorGearRatio;
    clampHoodTarget();
  }

  public void increaseHoodAngle() {
    hoodTarget += (1. / 360.) * HoodConstants.kEncoderGearRatio * HoodConstants.kMotorGearRatio;
    clampHoodTarget();
  }

  public void decreaseHoodAngle() {
    hoodTarget -= (1. / 360.) * HoodConstants.kEncoderGearRatio * HoodConstants.kMotorGearRatio;
    clampHoodTarget();
  }

  public void setHoodTarget(double target) {
    hoodTarget = target;
    clampHoodTarget();
  }

  public double getHoodTarget() {
    return hoodTarget;
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

  public void setShooterMap() {
    double distanceToHub = m_swerveSubsystem.getDistanceToHub();
    // set hood based on distance
    hoodTarget = hoodAngleMap.get(distanceToHub);
    clampHoodTarget();
    // set shooter based on distance
    shooterVelocityTarget = shooterSpeedMap.get(distanceToHub);
  }

  public void setShooterMapOnly() {
    double distanceToHub = m_swerveSubsystem.getDistanceToHub();
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

  /** Drive hood down at low duty cycle, bypassing PID. Works even with bad encoder. */
  public void nudgeHoodDown() {
    hoodManualOverride = true;
    hoodPrimaryMotor.set(-0.05);
  }

  /** Stop manual override, return to PID control. */
  public void stopHoodNudge() {
    hoodManualOverride = false;
    hoodPrimaryMotor.set(0);
  }

  /** @return Primary motor for simulation access */
  public SparkMax getPrimaryHoodMotor() {
    return hoodPrimaryMotor;
  }

  /** @return Motor Rotations (corrected for calibration offset) */
  public double getHoodMotorRotations() {
    return hoodEncoder.getPosition() + hoodCalibrationOffset;
  }

  /** @return Angle in Rad (corrected for calibration offset) */
  public double getHoodAngle() {
    return 2 * Math.PI * ((hoodEncoder.getPosition() + hoodCalibrationOffset - HoodConstants.kEncoderOffset) / (HoodConstants.kMotorGearRatio * HoodConstants.kEncoderGearRatio));
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

    velocityEntryHood.setDouble(getHoodVelocity());
    positionEntryHood.setDouble(getHoodMotorRotations());
    targetEntryHood.setDouble(hoodTarget);
    angleEntryHood.setDouble(Math.toDegrees(getHoodAngle()));
    primaryCurrentEntryHood.setDouble(getHoodPrimaryCurrent());
    secondaryCurrentEntryHood.setDouble(getHoodSecondaryCurrent());
    calibrationOffsetEntryHood.setDouble(hoodCalibrationOffset);
    encoderValidEntryHood.setBoolean(hoodEncoderValid);
    rawPositionEntryHood.setDouble(hoodEncoder.getPosition());
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    rampSetpoint();

    // Limit switch auto-re-zero (edge-triggered: fires once per activation)
    if (hoodLimitSwitch != null) {
      boolean pressed = !hoodLimitSwitch.get();
      if (pressed && !prevLimitSwitchPressed) {
        reZeroHood();
      }
      prevLimitSwitchPressed = pressed;
    }

    // Stall detection auto-re-zero: if targeting home, high current, no movement → stalled at hard stop.
    // Only fires when shooter is off — prevents interrupting active shots.
    // If hood stalls while shooting, rely on limit switch or manual re-zero (operator button 3).
    boolean isStalled = stallDebouncer.calculate(
        shooterVelocityTarget == 0
        && Math.abs(hoodTarget - HoodConstants.kEncoderOffset) < 0.05
        && getHoodPrimaryCurrent() > kStallCurrentThreshold
        && Math.abs(getHoodVelocity()) < kStallVelocityThreshold);
    // if (isStalled) {
    //   reZeroHood(); /

    updateNetworkTable();

    setShooterPIDVelocity();
    setHoodPIDAngle();
  }

  /** This method will be called once per scheduler run during simulation */
  @Override
  public void simulationPeriodic() {
    
  }
}