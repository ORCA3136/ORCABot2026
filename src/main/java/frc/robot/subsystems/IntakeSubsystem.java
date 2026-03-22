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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    FAULT
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

  final SparkClosedLoopController deployPIDController = intakeDeployMotor.getClosedLoopController();

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

  private int stallCounter = 0;

  private boolean pulseActive = false;
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

  /**
   * Requests the homing sequence. Transitions from UNHOMED → HOMING.
   * No-op if already HOMED or currently HOMING.
   */
  public void requestHoming() {
    if (DriverStation.isDisabled()) return;
    if (state == DeployState.FAULT) {
      DriverStation.reportWarning("IntakeSubsystem: Requested homing when FAULTED", false);
      return;
    }

    disableSoftLimits();
    state = DeployState.ZEROING;
    homingTimer.restart();
    DriverStation.reportWarning("IntakeSubsystem: Homing requested", false);
  }

  /**
   * Clears a fault and returns to UNHOMED. Call {@link #requestHoming()} after
   * to re-attempt homing.
   */
  public void clearFault() {
    if (state != DeployState.FAULT) return;
    faultReason = "";
    state = DeployState.UNHOMED;
    DriverStation.reportWarning("IntakeSubsystem: Fault cleared → UNHOMED", false);
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

    // Timeout
    if (homingTimer.hasElapsed(IntakeConstants.kHomingTimeoutSec)) {
      enterFault("Homing timed out after " + IntakeConstants.kHomingTimeoutSec + "s");
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
    // Stall detection (current spike)
    if (intakeDeployMotor.getOutputCurrent() > IntakeConstants.kStallCurrentThreshold) {
      stallCounter++;
    } else {
      stallCounter = 0;
    }

    if (stallCounter >= IntakeConstants.kStallCycles) {
      // Stall was detected for 1 second - faulting
      enterFault("Stall detected for intake deploy");
    }
  }

  /** Enters the FAULT state — stops motor, logs reason. */
  private void enterFault(String reason) {
    intakeDeployMotor.set(0);
    homingTimer.stop();
    faultReason = reason;
    state = DeployState.FAULT;
    DriverStation.reportError("IntakeSubsystem FAULT: " + reason, false);
  }

  /** Applies conservative soft limits (0 to kMaxExtension) before homing — protects hard stops. */
  private void disableSoftLimits() {
    SparkFlexConfig limitConfig = new SparkFlexConfig();
    limitConfig.softLimit
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimitEnabled(false);
    intakeDeployMotor.configure(limitConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** Applies forward/reverse hardware soft limits on the deploy motor. */
  private void setHardwareSoftLimits() {
    SparkFlexConfig limitConfig = new SparkFlexConfig();
    limitConfig.softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit((float) IntakeConstants.kMaxExtension)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(-1.0f);
    intakeDeployMotor.configure(limitConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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

  /** Sends the ramped position setpoint to the SparkFlex PID controller. */
  public void setPIDPosition() {
    if (state != DeployState.TARGETING) return;
    if (isPIDTargetReached()) {
      state = DeployState.SET;
      return;
    }
    deployPIDController.setSetpoint(rampedPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  /** Check if PID position reached */
  public boolean isPIDTargetReached() {
    boolean inTolerance = Math.abs(getIntakeDeployPosition() - rampedPosition) < IntakeConstants.kTargetTolerance;
    if (!inTolerance) PIDTimer.reset();
    
    return PIDTimer.advanceIfElapsed(IntakeConstants.kPIDTimeoutSec);
  }

  /** Sets the deploy target setpoint. Rejected if not HOMED. */
  public void setIntakeDeployTarget(Setpoint position) {
    Setpoint lastSetpoint = intakeDeployTarget;
    intakeDeployTarget = position;

    rampedPosition = getIntakeDeployPosition();

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
    disableSoftLimits();
    state = DeployState.MANUAL;
    // Block inward motion (negative speed) when limit switch is pressed
    if (speed < 0 && isLimitSwitchPressed()) {
      intakeDeployMotor.set(0);
      return;
    }
    intakeDeployMotor.set(speed / RobotConstants.kNeoVortexFreeSpeedRPM);
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
    SmartDashboard.putBoolean("Intake Limit Switch", isLimitSwitchPressed());
  }

  // ── Periodic ───────────────────────────────────────────────────────

  @Override
  public void periodic() {
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
      case FAULT:
        intakeDeployMotor.set(0);
        break;
    }
    checkIntakeDeployStall();
    updateNetworkTable();
  }

  @Override
  public void simulationPeriodic() {
    // Simulation not yet implemented for rack & pinion
  }
}