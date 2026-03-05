// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ClimberConfigs;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.NetworkTableNames;
import frc.robot.Constants.RobotConstants;

/**
 * Climber subsystem for tower climbing.
 *
 * Two NEO Vortex motors (SparkFlex, CAN 19/20) drive a two-hook arm through
 * a 318.18:1 reduction (125:1 MaxPlanetary × 28/11 sprocket chain).
 *
 * Primary motor (CAN 19): PID leader using relative encoder (handles multi-turn travel).
 * Secondary motor (CAN 20): Pure follower. Absolute encoder available for startup seed.
 *
 * Position is tracked in arm degrees (0° = stowed, ~180° = fully climbed).
 */
public class ClimberSubsystem extends SubsystemBase {

  // Motors
  private final SparkFlex climberPrimaryMotor = new SparkFlex(CanIdConstants.kClimberPrimaryCanId, MotorType.kBrushless);
  private final SparkFlex climberSecondaryMotor = new SparkFlex(CanIdConstants.kClimberSecondaryCanId, MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder climberEncoder = climberPrimaryMotor.getEncoder();
  private final AbsoluteEncoder absoluteEncoder = climberSecondaryMotor.getAbsoluteEncoder();

  // PID controller on primary motor
  private final SparkClosedLoopController pidController = climberPrimaryMotor.getClosedLoopController();

  // State
  private double targetDegrees = 0.0;
  private boolean isZeroed = false;
  private boolean manualOverride = false;
  private boolean safetyTripped = false;

  // Tunable position setpoints (read from SmartDashboard)
  private double stowedDegrees = ClimberConstants.kStowedDegrees;
  private double horizontalDegrees = ClimberConstants.kHorizontalDegrees;
  private double climbedDegrees = ClimberConstants.kClimbedDegrees;
  private double maxArmDegrees = ClimberConstants.kMaxArmDegrees;
  private double minArmDegrees = ClimberConstants.kMinArmDegrees;

  // Tunable PID gains
  private double tuneP = ClimberConstants.kP;
  private double tuneD = ClimberConstants.kD;
  private double tuneG = ClimberConstants.kG;
  private double tuneS = ClimberConstants.kS;

  // Tuning mode flag
  private boolean tuningEnabled = false;

  // NetworkTable entries
  private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  private final NetworkTable climberTable = networkTable.getTable(NetworkTableNames.Climber.kTable);

  private final NetworkTableEntry armDegreesEntry = climberTable.getEntry(NetworkTableNames.Climber.kArmDegrees);
  private final NetworkTableEntry targetDegreesEntry = climberTable.getEntry(NetworkTableNames.Climber.kTargetDegrees);
  private final NetworkTableEntry errorEntry = climberTable.getEntry(NetworkTableNames.Climber.kError);
  private final NetworkTableEntry motorRotationsEntry = climberTable.getEntry(NetworkTableNames.Climber.kMotorRotations);
  private final NetworkTableEntry leftOutputEntry = climberTable.getEntry(NetworkTableNames.Climber.kLeftMotorOutput);
  private final NetworkTableEntry rightOutputEntry = climberTable.getEntry(NetworkTableNames.Climber.kRightMotorOutput);
  private final NetworkTableEntry primaryCurrentEntry = climberTable.getEntry(NetworkTableNames.Climber.kPrimaryCurrent);
  private final NetworkTableEntry secondaryCurrentEntry = climberTable.getEntry(NetworkTableNames.Climber.kSecondaryCurrent);
  private final NetworkTableEntry absEncoderEntry = climberTable.getEntry(NetworkTableNames.Climber.kAbsEncoderRaw);
  private final NetworkTableEntry isZeroedEntry = climberTable.getEntry(NetworkTableNames.Climber.kIsZeroed);
  private final NetworkTableEntry atSetpointEntry = climberTable.getEntry(NetworkTableNames.Climber.kAtSetpoint);
  private final NetworkTableEntry stateEntry = climberTable.getEntry(NetworkTableNames.Climber.kState);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climberPrimaryMotor.configure(ClimberConfigs.climberPrimaryMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climberSecondaryMotor.configure(ClimberConfigs.climberSecondaryMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Zero the relative encoder — arm must start in stowed position
    zeroClimber();

    // Initialize tuning dashboard entries
    SmartDashboard.putNumber("Climber/Tuning/kP", tuneP);
    SmartDashboard.putNumber("Climber/Tuning/kD", tuneD);
    SmartDashboard.putNumber("Climber/Tuning/kG", tuneG);
    SmartDashboard.putNumber("Climber/Tuning/kS", tuneS);
    SmartDashboard.putNumber("Climber/Tuning/StowedDeg", stowedDegrees);
    SmartDashboard.putNumber("Climber/Tuning/HorizontalDeg", horizontalDegrees);
    SmartDashboard.putNumber("Climber/Tuning/ClimbedDeg", climbedDegrees);
    SmartDashboard.putNumber("Climber/Tuning/MaxDeg", maxArmDegrees);
    SmartDashboard.putNumber("Climber/Tuning/MinDeg", minArmDegrees);
    SmartDashboard.putBoolean("Climber/Tuning/Enabled", tuningEnabled);
  }

  // ── Encoder Management ──────────────────────────────────────────────

  /**
   * Zero the relative encoder. Arm MUST be at stowed position when called.
   * Seeds position to 0° (stowed).
   */
  public void zeroClimber() {
    REVLibError err = climberEncoder.setPosition(0);
    if (err == REVLibError.kOk) {
      targetDegrees = 0.0;
      isZeroed = true;
    } else {
      isZeroed = false;
      System.err.println("ClimberSubsystem: FAILED to zero encoder: " + err);
    }
  }

  /** @return Current arm position in degrees (0° = stowed). */
  public double getArmDegrees() {
    return climberEncoder.getPosition();
  }

  /** @return Raw absolute encoder reading (0-1 rotations) for diagnostics. */
  public double getAbsoluteEncoderRaw() {
    return absoluteEncoder.getPosition();
  }

  /** @return true if the encoder has been zeroed since startup. */
  public boolean isZeroed() {
    return isZeroed;
  }

  // ── Position Control ────────────────────────────────────────────────

  /**
   * Set target arm position using onboard SparkFlex PID.
   * Clamped to software limits.
   * @param degrees Target arm angle in degrees
   */
  public void setArmPosition(double degrees) {
    manualOverride = false;
    degrees = MathUtil.clamp(degrees, minArmDegrees, maxArmDegrees);
    targetDegrees = degrees;
  }

  /** @return true if the arm is within tolerance of the target. */
  public boolean atSetpoint() {
    return atSetpoint(ClimberConstants.kSetpointToleranceDeg);
  }

  /** @return true if the arm is within the specified tolerance of the target. */
  public boolean atSetpoint(double toleranceDeg) {
    return Math.abs(getArmDegrees() - targetDegrees) < toleranceDeg;
  }

  /** @return Current target in degrees. */
  public double getTargetDegrees() {
    return targetDegrees;
  }

  /** Calculate gravity + static feedforward for the arm. */
  public double calculateFeedForward() {
    return tuneS * Math.signum(targetDegrees - getArmDegrees())
         + tuneG * Math.cos(Math.toRadians(getArmDegrees()));
  }

  // ── Manual Duty-Cycle Control ───────────────────────────────────────

  /**
   * Direct duty-cycle control with software limits enforced.
   * Takes an RPM-scale value and normalizes to [-1, 1].
   * @param speed RPM-scale value (positive = extend, negative = retract)
   */
  public void setManualDutyCycle(double speed) {
    if (safetyTripped) { climberPrimaryMotor.set(0); return; }
    manualOverride = true;
    double armDeg = getArmDegrees();
    // Enforce limits even in manual mode
    if (isZeroed) {
      if (armDeg >= maxArmDegrees && speed > 0) { speed = 0; }
      if (armDeg <= minArmDegrees && speed < 0) { speed = 0; }
    }
    climberPrimaryMotor.set(speed / RobotConstants.kNeoVortexFreeSpeedRPM);
  }

  /** Stop manual control and hold current position with PID. */
  public void stopManual() {
    manualOverride = false;
    targetDegrees = getArmDegrees();
  }

  // ── Position Getters ────────────────────────────────────────────────

  public double getStowedDegrees() { return stowedDegrees; }
  public double getHorizontalDegrees() { return horizontalDegrees; }
  public double getClimbedDegrees() { return climbedDegrees; }

  // ── Position Capture Helpers ────────────────────────────────────────

  /** Capture current arm position as the "horizontal" setpoint and update dashboard. */
  public void captureCurrentAsHorizontal() {
    horizontalDegrees = getArmDegrees();
    SmartDashboard.putNumber("Climber/Tuning/HorizontalDeg", horizontalDegrees);
  }

  /** Capture current arm position as the "climbed" setpoint and update dashboard. */
  public void captureCurrentAsClimbed() {
    climbedDegrees = getArmDegrees();
    SmartDashboard.putNumber("Climber/Tuning/ClimbedDeg", climbedDegrees);
  }

  /** Capture current arm position as the max travel limit and update dashboard. */
  public void captureCurrentAsMax() {
    maxArmDegrees = getArmDegrees();
    SmartDashboard.putNumber("Climber/Tuning/MaxDeg", maxArmDegrees);
  }

  // ── State Estimation ────────────────────────────────────────────────

  /** @return Human-readable state string for dashboard display. */
  public String getStateString() {
    double armDeg = getArmDegrees();
    if (armDeg < stowedDegrees + 10) return "STOWED";
    if (Math.abs(armDeg - horizontalDegrees) < 15) return "HORIZONTAL";
    if (armDeg > climbedDegrees - 15) return "CLIMBED";
    return "CLIMBING";
  }

  // ── Simulation Access ───────────────────────────────────────────────

  /** @return Primary motor for simulation access. */
  public SparkFlex getPrimaryMotor() {
    return climberPrimaryMotor;
  }

  // ── Tuning ──────────────────────────────────────────────────────────

  /** Read tuning values from SmartDashboard and apply PID changes. */
  private void updateTuningValues() {
    tuningEnabled = SmartDashboard.getBoolean("Climber/Tuning/Enabled", false)
        && !DriverStation.isFMSAttached();
    if (!tuningEnabled) return;

    double newP = SmartDashboard.getNumber("Climber/Tuning/kP", tuneP);
    double newD = SmartDashboard.getNumber("Climber/Tuning/kD", tuneD);
    tuneG = SmartDashboard.getNumber("Climber/Tuning/kG", tuneG);
    tuneS = SmartDashboard.getNumber("Climber/Tuning/kS", tuneS);

    // Clamp PID gains to safe ranges
    newP = MathUtil.clamp(newP, 0, 0.5);
    newD = MathUtil.clamp(newD, 0, 2.0);

    // Only reconfigure if PID gains changed (avoid CAN spam)
    if (newP != tuneP || newD != tuneD) {
      tuneP = newP;
      tuneD = newD;
      // Build a new config with updated PID
      var updatedConfig = new com.revrobotics.spark.config.SparkFlexConfig();
      updatedConfig.closedLoop
          .pid(tuneP, ClimberConstants.kI, tuneD);
      climberPrimaryMotor.configure(updatedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    stowedDegrees = MathUtil.clamp(SmartDashboard.getNumber("Climber/Tuning/StowedDeg", stowedDegrees), -10, 30);
    horizontalDegrees = MathUtil.clamp(SmartDashboard.getNumber("Climber/Tuning/HorizontalDeg", horizontalDegrees), 30, 150);
    climbedDegrees = MathUtil.clamp(SmartDashboard.getNumber("Climber/Tuning/ClimbedDeg", climbedDegrees), 90, 200);
    maxArmDegrees = MathUtil.clamp(SmartDashboard.getNumber("Climber/Tuning/MaxDeg", maxArmDegrees), 0, 360);
    minArmDegrees = MathUtil.clamp(SmartDashboard.getNumber("Climber/Tuning/MinDeg", minArmDegrees), -30, 90);
  }

  // ── Telemetry ───────────────────────────────────────────────────────

  private void publishTelemetry() {
    double armDeg = getArmDegrees();
    armDegreesEntry.setDouble(armDeg);
    targetDegreesEntry.setDouble(targetDegrees);
    errorEntry.setDouble(targetDegrees - armDeg);
    motorRotationsEntry.setDouble(climberEncoder.getPosition()
        * ClimberConstants.kTotalReduction / 360.0); // convert back to raw motor rotations
    leftOutputEntry.setDouble(climberPrimaryMotor.getAppliedOutput());
    rightOutputEntry.setDouble(climberSecondaryMotor.getAppliedOutput());
    primaryCurrentEntry.setDouble(climberPrimaryMotor.getOutputCurrent());
    secondaryCurrentEntry.setDouble(climberSecondaryMotor.getOutputCurrent());
    absEncoderEntry.setDouble(absoluteEncoder.getPosition());
    isZeroedEntry.setBoolean(isZeroed);
    atSetpointEntry.setBoolean(atSetpoint());
    stateEntry.setString(getStateString());
    // Motor temperatures and extra state
    climberTable.getEntry("Primary Temp C").setDouble(climberPrimaryMotor.getMotorTemperature());
    climberTable.getEntry("Secondary Temp C").setDouble(climberSecondaryMotor.getMotorTemperature());
    climberTable.getEntry("FeedForward").setDouble(calculateFeedForward());
    climberTable.getEntry("Manual Override").setBoolean(manualOverride);
    climberTable.getEntry("Safety Tripped").setBoolean(safetyTripped);
  }

  // ── Periodic ────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    double armDeg = getArmDegrees();

    // HARD SAFETY — cut motor if past limits, clamp target to prevent oscillation
    if (isZeroed && (armDeg > maxArmDegrees || armDeg < minArmDegrees)) {
      climberPrimaryMotor.set(0);
      targetDegrees = MathUtil.clamp(armDeg, minArmDegrees, maxArmDegrees);
      manualOverride = false;
      safetyTripped = true;
    } else {
      safetyTripped = false;
      if (!manualOverride) {
        // Run PID to hold/reach target
        pidController.setSetpoint(targetDegrees, ControlType.kPosition, ClosedLoopSlot.kSlot0, calculateFeedForward());
      }
    }

    updateTuningValues();
    publishTelemetry();
  }

  @Override
  public void simulationPeriodic() {
  }
}
