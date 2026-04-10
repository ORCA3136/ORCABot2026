// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfigs;
import frc.robot.Constants.*;

/**
 * Rack & pinion intake subsystem for Deuce (Robot 2).
 *
 * Deploys linearly via a 25:1 gearbox + 18t→22t sprocket + 12t→32t rack.
 * Full travel: 66.75 motor rotations.
 * Position control uses the SparkFlex relative encoder (motor rotations from home).
 *
 * Homing uses a limit switch at the retracted position (DIO 2) as the
 * authoritative zero reference. Falls back to stall detection when
 * {@code kLimitSwitchInstalled} is false.
 */
public class IntakeSubsystem extends SubsystemBase {

  // ── Deploy state machine ────────────────────────────────────────────
  public enum DeployState {
    UNHOMED,
    ZEROING,
    TARGETING,
    SET,
    MANUAL,
    STALL_RECOVERY
  }

  public enum Setpoint {
    kRetracted,
    kShuttle,   // Center point for shuttle pulse feeding (was kPartial)
    kExtended;
  }

  // ── Hardware ─────────────────────────────────────────────────────────
  final SparkFlex intakeMotor = new SparkFlex(CanIdConstants.kIntakeCanId, MotorType.kBrushless);
  final SparkFlex intakeDeployMotor = new SparkFlex(CanIdConstants.kIntakeDeployCanId, MotorType.kBrushless);

  final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
  final RelativeEncoder intakeDeployEncoder = intakeDeployMotor.getEncoder();
  final AbsoluteEncoder intakeDeployAbsEncoder = intakeDeployMotor.getAbsoluteEncoder();

  private final SparkClosedLoopController deployPIDController = intakeDeployMotor.getClosedLoopController();

  private final DigitalInput homeLimitSwitch = new DigitalInput(DioConstants.kIntakeHomeLimitSwitchPort);

  // ── NetworkTables ───────────────────────────────────────────────────
  final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  final NetworkTable intakeTable = networkTable.getTable(NetworkTableNames.Intake.kTable);
  final NetworkTable intakeDeployTable = networkTable.getTable(NetworkTableNames.IntakeDeploy.kTable);

  private final NetworkTableEntry intakeVelocityEntry = intakeTable.getEntry(NetworkTableNames.Intake.kVelocityRPM);
  private final NetworkTableEntry intakeCurrentEntry = intakeTable.getEntry(NetworkTableNames.Intake.kCurrentAmps);
  private final NetworkTableEntry deployVelocityEntry = intakeDeployTable.getEntry(NetworkTableNames.IntakeDeploy.kVelocityRPM);
  private final NetworkTableEntry deployPositionEntry = intakeDeployTable.getEntry(NetworkTableNames.IntakeDeploy.kPositionRotations);
  private final NetworkTableEntry deployCurrentEntry = intakeDeployTable.getEntry(NetworkTableNames.IntakeDeploy.kCurrentAmps);
  private final NetworkTableEntry deployTargetEntry = intakeDeployTable.getEntry(NetworkTableNames.IntakeDeploy.kTarget);
  private final NetworkTableEntry rampedPositionEntry = intakeDeployTable.getEntry(NetworkTableNames.IntakeDeploy.kRampedPosition);
  private final NetworkTableEntry stateEntry = intakeDeployTable.getEntry(NetworkTableNames.IntakeDeploy.kState);
  private final NetworkTableEntry faultReasonEntry = intakeDeployTable.getEntry(NetworkTableNames.IntakeDeploy.kFaultReason);
  private final NetworkTableEntry limitSwitchEntry = intakeDeployTable.getEntry(NetworkTableNames.IntakeDeploy.kLimitSwitch);
  private final NetworkTableEntry absEncoderRawEntry = intakeDeployTable.getEntry(NetworkTableNames.IntakeDeploy.kAbsEncoderRaw);

  // ── State ───────────────────────────────────────────────────────────
  private DeployState state = DeployState.UNHOMED;
  private String faultReason = "";
  private final Timer homingTimer = new Timer();
  private final Timer PIDTimer = new Timer();
  private boolean intakeRunning = false;

  private int stallCounter = 0;
  private int stallRetryCount = 0;
  private double stallRecoveryTarget = 0;
  private final Timer stallRecoveryTimer = new Timer();

  // ── Fuel detection state ────────────────────────────────────────
  private int     fuelDetectCounter = 0;
  private boolean fuelDetected      = false;
  private boolean fuelDetectArmed   = true;
  private final Timer noFuelTimer   = new Timer();

  private boolean autoStopped       = false;

  private boolean pulseActive = false;
  private boolean hardwareSoftLimitsActive = false;
  private Setpoint intakeDeployTarget = Setpoint.kRetracted;
  private double rampedPosition = IntakeConstants.kRetractedPosition;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(SwerveSubsystem swerveSubsystem) {
    intakeMotor.configure(IntakeConfigs.intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeDeployMotor.configure(IntakeConfigs.intakeDeployMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    if (!IntakeConstants.kLimitSwitchInstalled) {
      DriverStation.reportWarning("IntakeSubsystem: Limit switch NOT installed — using stall-detection fallback for homing", false);
    }

    // Boot-time limit switch check — if retracted at boot, home immediately (no motors move)
    if (IntakeConstants.kLimitSwitchInstalled && isLimitSwitchPressed()) {
      intakeDeployEncoder.setPosition(0);
      setHardwareSoftLimits();
      state = DeployState.SET;
      DriverStation.reportWarning("IntakeSubsystem: Limit switch detected at boot — auto-homed", false);
    } else {
      // Apply conservative soft limits for hard-stop protection even while unhomed
      disableSoftLimits();
      DriverStation.reportWarning("IntakeSubsystem: Limit switch NOT detected at boot — manual homing required", false);
    }
  }

  // ── State machine ───────────────────────────────────────────────────

  /** @return Current deploy state. */
  public DeployState getDeployState() {
    return state;
  }

  /** @return True if the intake is homed and ready for position commands. */
  public boolean isHomed() {
    return state == DeployState.SET;
  }

  /** @return True if the limit switch is pressed (NO switch with pull-up: pressed = get() returns false). */
  public boolean isLimitSwitchPressed() {
    return !homeLimitSwitch.get();
  }

  public void runOrStopIntakeRoller() {
    intakeRunning = !intakeRunning;
    if (intakeRunning) {
      setIntakeDutyCycle(6500);
      noFuelTimer.restart();
    }
    if (!intakeRunning) {
      setIntakeDutyCycle(0);
      noFuelTimer.stop();
      noFuelTimer.reset();
    }
  }

  /** Stops the intake roller and resets the running flag. */
  public void stopIntakeRoller() {
    intakeRunning = false;
    setIntakeDutyCycle(0);
    noFuelTimer.stop();
    noFuelTimer.reset();
  }

  /** @return true if the intake roller is currently running. */
  public boolean isIntakeRunning() {
    return intakeRunning;
  }

  /** @return seconds since the roller started or since last fuel detection. */
  public double getNoFuelElapsedSeconds() {
    return noFuelTimer.get();
  }

  /**
   * Returns true for exactly one scheduler cycle when the roller
   * is automatically stopped due to the no-fuel timeout.
   */
  public boolean wasAutoStopped() {
    return autoStopped;
  }

  /**
   * Requests the homing sequence. Transitions from UNHOMED → HOMING.
   * No-op if already HOMED or currently HOMING.
   */
  public void requestHoming() {
    if (DriverStation.isDisabled()) return;

    disableSoftLimits();
    state = DeployState.ZEROING;
    homingTimer.restart();
    DriverStation.reportWarning("IntakeSubsystem: Homing requested", false);
  }

  /** Runs each cycle during HOMING state. */
  private void updateHoming() {
    // Drive slowly toward retracted (inward)
    intakeDeployMotor.set(IntakeConstants.kHomingDutyCycle);

    // Primary: limit switch
    if (IntakeConstants.kLimitSwitchInstalled && isLimitSwitchPressed()) {
      completeHoming();
      return;
    }
  }

  /** Called when homing succeeds — zeros encoder, applies soft limits, transitions to HOMED. */
  private void completeHoming() {
    intakeDeployMotor.set(0);
    intakeDeployEncoder.setPosition(0);
    rampedPosition = 0;
    homingTimer.stop();

    // Apply hardware soft limits now that we have a known zero
    setHardwareSoftLimits();

    if (intakeDeployTarget != Setpoint.kRetracted) state = DeployState.TARGETING;
    else state = DeployState.SET;
    DriverStation.reportWarning("IntakeSubsystem: Homing complete → HOMED", false);
  }

  /** Runs each cycle to check if intake deploy is stalled */
  private void checkIntakeDeployStall() {
    // Only check for stalls when the motor is actively being driven
    if (state != DeployState.TARGETING && state != DeployState.MANUAL && state != DeployState.STALL_RECOVERY) {
      stallCounter = 0;
      return;
    }

    // Stall detection (current spike)
    if (intakeDeployMotor.getOutputCurrent() > IntakeConstants.kStallCurrentThreshold) {
      stallCounter++;
    } else {
      stallCounter = 0;
    }

    if (stallCounter >= IntakeConstants.kStallCycles) {
      if (stallRetryCount < IntakeConstants.kMaxStallRetries) {
        // Attempt stall recovery — reverse direction to dislodge fuel
        stallRetryCount++;
        stallCounter = 0;

        // Reverse: if heading toward extended (positive), go back; if toward retracted, go forward
        double currentPos = getIntakeDeployPosition();
        double target = calculatePosition();
        if (currentPos < target) {
          // Was extending → reverse inward
          stallRecoveryTarget = currentPos - IntakeConstants.kStallReverseRotations;
        } else {
          // Was retracting → reverse outward
          stallRecoveryTarget = currentPos + IntakeConstants.kStallReverseRotations;
        }
        // Clamp within soft limits
        stallRecoveryTarget = Math.max(IntakeConstants.kRetractedPosition,
            Math.min(stallRecoveryTarget, IntakeConstants.kMaxExtension));

        state = DeployState.STALL_RECOVERY;
        stallRecoveryTimer.restart();
        DriverStation.reportWarning("IntakeSubsystem: Stall recovery attempt "
            + stallRetryCount + "/" + IntakeConstants.kMaxStallRetries, false);
      }
    }
  }

  /** Drives the deploy motor to the recovery target, then resumes TARGETING. */
  private void updateStallRecovery() {
    deployPIDController.setSetpoint(stallRecoveryTarget, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0);

    boolean posReached = Math.abs(getIntakeDeployPosition() - stallRecoveryTarget) < IntakeConstants.kTargetTolerance;
    boolean timedOut = stallRecoveryTimer.hasElapsed(IntakeConstants.kStallRecoveryTimeoutSec);

    if (posReached || timedOut) {
      stallCounter = 0;
      rampedPosition = getIntakeDeployPosition();
      state = DeployState.TARGETING;
      DriverStation.reportWarning("IntakeSubsystem: Stall recovery complete, resuming", false);
    }
  }

  /**
   * Returns true when the deploy motor current exceeds the stall threshold
   * and the motor is actively being driven.
   */
  public boolean isDeployStallWarning() {
    if (state != DeployState.TARGETING && state != DeployState.MANUAL && state != DeployState.STALL_RECOVERY) return false;
    return intakeDeployMotor.getOutputCurrent() > IntakeConstants.kStallCurrentThreshold;
  }

  /** Disables soft limits on the deploy motor. Skips if already disabled. */
  private void disableSoftLimits() {
    if (!hardwareSoftLimitsActive) return;
    SparkFlexConfig limitConfig = new SparkFlexConfig();
    limitConfig.softLimit
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimitEnabled(false);
    intakeDeployMotor.configure(limitConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    hardwareSoftLimitsActive = false;
  }

  /** Applies forward/reverse hardware soft limits on the deploy motor. Skips if already active. */
  private void setHardwareSoftLimits() {
    if (hardwareSoftLimitsActive) return;
    SparkFlexConfig limitConfig = new SparkFlexConfig();
    limitConfig.softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit((float) IntakeConstants.kMaxExtension)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(-0.25f);
    intakeDeployMotor.configure(limitConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    hardwareSoftLimitsActive = true;
  }

  // ── Position control ───────────────────────────────────────────────

  /** Calculates the target position in motor rotations from the current setpoint. */
  public double calculatePosition() {
    double targetPosition;

    switch (intakeDeployTarget) {
      case kExtended:
        targetPosition = IntakeConstants.kExtendedPosition;
        break;
      case kShuttle:
        targetPosition = IntakeConstants.kShuttleCenter;
        break;
      case kRetracted:
      default:
        targetPosition = IntakeConstants.kRetractedPosition;
        break;
    }

    // Add oscillation offset when pulsing
    if (pulseActive) {
      targetPosition += IntakeConstants.kPulseAmplitude
          * Math.sin(Timer.getTimestamp() * 2 * Math.PI * IntakeConstants.kPulseFrequencyHz);
    }

    // Enforce soft limits
    targetPosition = Math.max(IntakeConstants.kRetractedPosition, Math.min(targetPosition, IntakeConstants.kMaxExtension));

    return targetPosition;
  }

  /** Ramps the position setpoint toward the target for smoother motion. */
  private void rampPosition() {
    // Slow retract: gradually decrease position toward retracted
    if (slowRetractActive && !pulseActive) {
      rampedPosition = Math.max(
          rampedPosition - IntakeConstants.kFeedRetractRate,
          IntakeConstants.kRetractedPosition);
      return;
    }

    double target = calculatePosition();
    if (rampedPosition < target) {
      rampedPosition = Math.min(rampedPosition + IntakeConstants.kExtendRampRate, target);
    } else if (rampedPosition > target) {
      rampedPosition = Math.max(rampedPosition - IntakeConstants.kRetractRampRate, target);
    }
  }

  /**
   * Computes an arbitrary feedforward value to counteract gravity during extend.
   * Only applies a positive offset when extending (positive error beyond deadband).
   * Retracting needs no FF because gravity assists.
   */
  private double computeArbFF() {
    double error = rampedPosition - getIntakeDeployPosition();
    if (error > IntakeConstants.kArbFFDeadband) {
      return IntakeConstants.kExtendArbFF;
    }
    return 0.0;
  }

  /** Sends the ramped position setpoint to the SparkFlex PID controller. */
  public void setPIDPosition() {
    if (state != DeployState.TARGETING) return;
    if (isPIDTargetReached()) {
      state = DeployState.SET;
      return;
    }
    deployPIDController.setSetpoint(rampedPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0, computeArbFF());
  }

  /** Check if PID position reached */
  public boolean isPIDTargetReached() {
    boolean inTolerance = Math.abs(getIntakeDeployPosition() - rampedPosition) < IntakeConstants.kTargetTolerance;
    if (!inTolerance) PIDTimer.reset();
    
    return PIDTimer.advanceIfElapsed(IntakeConstants.kPIDTimeoutSec);
  }

  public SparkClosedLoopController getDeployClosedLoopController() {
    return deployPIDController;
  }

  /** Sets the deploy target setpoint. Rejected if not HOMED. */
  public void setIntakeDeployTarget(Setpoint position) {
    Setpoint lastSetpoint = intakeDeployTarget;
    intakeDeployTarget = position;

    rampedPosition = getIntakeDeployPosition();
    stallCounter = 0;
    stallRetryCount = 0;

    PIDTimer.restart();
    // state = DeployState.TARGETING;
    // setHardwareSoftLimits();
    if (state == DeployState.SET && lastSetpoint != position) {
      state = DeployState.TARGETING;
      setHardwareSoftLimits();
    }
    else if (state == DeployState.MANUAL) {
      state = DeployState.TARGETING;
      setHardwareSoftLimits();
    }
  }

  /** @return Current deploy position in motor rotations from home. */
  public double getIntakePosition() {
    return intakeDeployEncoder.getPosition();
  }

  /** @return Deploy position (alias for compatibility). */
  public double getIntakeDeployPosition() {
    return getIntakePosition();
  }

  // ── Slow retract for feeding ──────────────────────────────────────

  private boolean slowRetractActive = false;

  /**
   * Enable/disable slow retraction. When active, the ramped position target
   * gradually decreases each cycle, pulling fuel toward the conveyor.
   * Stops at kRetracted (0). Disable to hold current position.
   */
  public void slowRetract(boolean enable) {
    slowRetractActive = enable;
    if (enable && state == DeployState.SET) {
      state = DeployState.TARGETING;
      // Kickstart: give PID initial error to overcome static friction
      rampedPosition = Math.max(
          rampedPosition - 0.5,
          IntakeConstants.kRetractedPosition);
    }
  }

  /** @return true if slow retract is currently active. */
  public boolean isSlowRetractActive() {
    return slowRetractActive;
  }

  // ── Shuttle pulse for feeding ───────────────────────────────────────

  /** Enable/disable shuttle pulse (in/out motion for fuel agitation). */
  public void pulse(boolean enable) {
    pulseActive = enable;
  }

  /** Alias for backwards compatibility with oscillate calls. */
  public void ocillateIntake(boolean oscillate) {
    pulse(oscillate);
  }

  // ── Intake roller ──────────────────────────────────────────────────

  /**
   * Sets intake roller motor output. Takes an RPM-scale value and normalizes it to [-1, 1]
   * duty cycle by dividing by the NEO Vortex free speed (6500 RPM).
   * @param speed RPM-scale value (e.g. 4000 for intaking, -3000 for outtaking)
   */
  public void setIntakeDutyCycle(double speed) {
    intakeMotor.set(speed / RobotConstants.kNeoVortexFreeSpeedRPM);
  }

  /** @return Intake roller velocity in RPM. */
  public double getIntakeVelocity() {
    return intakeEncoder.getVelocity();
  }

  /**
   * Sets intake deploy motor output directly (duty cycle scale).
   * Rejected in FAULT or HOMING states. Allowed in UNHOMED for manual jog.
   * @param speed RPM-scale value normalized by Vortex free speed
   */
  public void setIntakeDeployDutyCycle(double speed) {
    if (state != DeployState.MANUAL) {
      disableSoftLimits();
      state = DeployState.MANUAL;
    }
    // Block inward motion (negative speed) when limit switch is pressed
    if (speed < 0 && isLimitSwitchPressed()) {
      intakeDeployMotor.set(0);
      return;
    }
    intakeDeployMotor.set(speed / RobotConstants.kNeoVortexFreeSpeedRPM);
  }

  public void stopMotor() {
    setIntakeDeployDutyCycle(0);
  }

  /** @return Deploy motor velocity in RPM. */
  public double getIntakeDeployVelocity() {
    return intakeDeployEncoder.getVelocity();
  }

  // ── Motor accessors ────────────────────────────────────────────────

  /** @return Intake roller motor for simulation access. */
  public SparkFlex getIntakeMotor() {
    return intakeMotor;
  }

  /** @return Deploy motor for simulation access. */
  public SparkFlex getDeployMotor() {
    return intakeDeployMotor;
  }

  /** Get the current applied voltage (both motors combined). */
  public double getVoltage() {
    return intakeDeployMotor.getAppliedOutput() * intakeDeployMotor.getBusVoltage()
        + intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
  }

  public double getDeployVoltage() {
    return intakeDeployMotor.getAppliedOutput() * intakeDeployMotor.getBusVoltage();
  }

  // ── NetworkTables ──────────────────────────────────────────────────

  public void updateNetworkTable() {
    intakeVelocityEntry.setDouble(getIntakeVelocity());
    intakeCurrentEntry.setDouble(intakeMotor.getOutputCurrent());
    deployCurrentEntry.setDouble(intakeDeployMotor.getOutputCurrent());
    deployVelocityEntry.setDouble(getIntakeDeployVelocity());
    deployPositionEntry.setDouble(getIntakePosition());
    deployTargetEntry.setDouble(calculatePosition());
    rampedPositionEntry.setDouble(rampedPosition);
    stateEntry.setString(state.name());
    faultReasonEntry.setString(faultReason);
    limitSwitchEntry.setBoolean(isLimitSwitchPressed());
    absEncoderRawEntry.setDouble(intakeDeployAbsEncoder.getPosition());
    // Removed duplicate SmartDashboard write — already published as IntakeDeploy/LimitSwitch
  }

  // ── Fuel detection ─────────────────────────────────────────────

  /**
   * Monitors intake roller current for a fuel pickup spike.
   * Only checks when the roller is actively intaking (positive applied output).
   * Uses debounce (consecutive cycles) to reject transient noise.
   * Latches {@code fuelDetected} for one cycle per event; re-arms when current drops.
   */
  private void checkFuelDetection() {
    double rollerOutput = intakeMotor.getAppliedOutput();
    double rollerCurrent = intakeMotor.getOutputCurrent();

    // Only detect when roller is actively intaking
    if (rollerOutput <= 0.01) {
      fuelDetectCounter = 0;
      fuelDetectArmed = true;
      fuelDetected = false;
      return;
    }

    if (rollerCurrent > IntakeConstants.kFuelDetectCurrentAmps) {
      fuelDetectCounter++;
    } else {
      fuelDetectCounter = 0;
      fuelDetectArmed = true;
    }

    // One-shot trigger: fire once per spike event
    if (fuelDetectCounter >= IntakeConstants.kFuelDetectDebounceCycles && fuelDetectArmed) {
      fuelDetected = true;
      fuelDetectArmed = false;
      noFuelTimer.restart(); // reset the no-fuel timeout
    } else {
      fuelDetected = false;
    }
  }

  /**
   * Returns true for exactly one scheduler cycle when fuel is detected
   * via a current spike on the intake roller.
   */
  public boolean isFuelDetected() {
    return fuelDetected;
  }

  /**
   * Auto-stops the intake roller if it has been running for
   * {@link IntakeConstants#kNoFuelTimeoutSec} without detecting fuel.
   * Sets {@code autoStopped} for one cycle so RobotContainer can trigger feedback.
   */
  private void checkNoFuelTimeout() {
    autoStopped = false;
    if (intakeRunning && noFuelTimer.hasElapsed(IntakeConstants.kNoFuelTimeoutSec)) {
      stopIntakeRoller();
      autoStopped = true;
    }
  }

  // ── Limit switch re-zero ───────────────────────────────────────────

  /**
   * Re-zeros the deploy encoder whenever the limit switch is pressed during
   * normal operation (TARGETING, SET, MANUAL). Corrects encoder drift so
   * position stays accurate across extend/retract cycles.
   * Skips during ZEROING — homing handles that via completeHoming().
   */
  private void checkLimitSwitchRezero() {
    if (!IntakeConstants.kLimitSwitchInstalled) return;
    if (!isLimitSwitchPressed()) return;
    if (state == DeployState.ZEROING) return;
    // Don't reset ramp when PID is actively extending away from home
    if (state == DeployState.TARGETING && intakeDeployTarget != Setpoint.kRetracted) return;

    intakeDeployEncoder.setPosition(0);
    rampedPosition = 0;
  }

  // ── Periodic ───────────────────────────────────────────────────────

  @Override
  public void periodic() {
    checkLimitSwitchRezero();
    switch (state) {
      case UNHOMED:
        // Idle — motor stopped, waiting for requestHoming()
        requestHoming();
        break;
      case ZEROING:
        updateHoming();
        break;
      case TARGETING:
        rampPosition();
        setPIDPosition();
        break;
      case SET:
        // Intake is in correct spot - stop motor
        intakeDeployMotor.set(0);
        break;
      case MANUAL:
        // Driver is currently controlling intake
        // Do not call ANY motor commands
        break;
      case STALL_RECOVERY:
        updateStallRecovery();
        break;
      default:
        intakeDeployMotor.set(0);
        break;
    }
    checkIntakeDeployStall();
    checkFuelDetection();
    checkNoFuelTimeout();
    updateNetworkTable();
  }

  @Override
  public void simulationPeriodic() {
    // Simulation not yet implemented for rack & pinion
  }
}