package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FuelPathConstants;
import frc.robot.RobotLogger;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Setpoint;

/**
 * Static factory methods for all fuel-path commands.
 *
 * One file = one "menu" of everything the intake -> conveyor -> kicker path can do.
 * Existing command classes (RunIntakeCommand, RunConveyorCommand, etc.) are the building
 * blocks these factories reuse.
 */
public final class FuelPathCommands {

  private FuelPathCommands() {} // Prevent instantiation

  // Temporary Command for Shooter

  public static Command shootToHub(ShooterSubsystem shooter) {
    return new ShootCommand(shooter);
  }

  // ── Individual: Intake ──────────────────────────────────────────────

  public static Command intakeIn(IntakeSubsystem intake) {
    return new RunIntakeCommand(intake, FuelPathConstants.kIntakeInStandard).withName("IntakeIn");
  }

  public static Command intakeInSlow(IntakeSubsystem intake) {
    return new RunIntakeCommand(intake, FuelPathConstants.kIntakeInSlow).withName("IntakeInSlow");
  }

  public static Command intakeInFast(IntakeSubsystem intake) {
    return new RunIntakeCommand(intake, FuelPathConstants.kIntakeInFast).withName("IntakeInFast");
  }

  public static Command intakeOut(IntakeSubsystem intake) {
    return new RunIntakeCommand(intake, FuelPathConstants.kIntakeOutStandard).withName("IntakeOut");
  }

  public static Command intakeOutSlow(IntakeSubsystem intake) {
    return new RunIntakeCommand(intake, FuelPathConstants.kIntakeOutSlow).withName("IntakeOutSlow");
  }

  public static Command intakeOutFast(IntakeSubsystem intake) {
    return new RunIntakeCommand(intake, FuelPathConstants.kIntakeOutFast).withName("IntakeOutFast");
  }

  // ── Individual: Conveyor ────────────────────────────────────────────

  public static Command conveyorIn(ConveyorSubsystem conveyor) {
    return new RunConveyorCommand(conveyor, FuelPathConstants.kConveyorIn).withName("ConveyorIn");
  }

  public static Command conveyorOut(ConveyorSubsystem conveyor) {
    return new RunConveyorCommand(conveyor, FuelPathConstants.kConveyorOut).withName("ConveyorOut");
  }

  // ── Individual: Kicker ──────────────────────────────────────────────

  public static Command kickerFeed(KickerSubsystem kicker) {
    return new RunKickerCommand(kicker, FuelPathConstants.kKickerFeed).withName("KickerFeed");
  }

  public static Command kickerOut(KickerSubsystem kicker) {
    return new RunKickerCommand(kicker, FuelPathConstants.kKickerOut).withName("KickerOut");
  }

  // ── Multi-subsystem compositions ────────────────────────────────────

  /** Intake roller + conveyor running together. */
  public static Command intakeAndConveyor(IntakeSubsystem intake, ConveyorSubsystem conveyor) {
    return Commands.parallel(
        new RunIntakeCommand(intake, FuelPathConstants.kIntakeInStandard),
        new RunConveyorCommand(conveyor, FuelPathConstants.kConveyorIn)
    ).withName("IntakeAndConveyor");
  }

  /** Conveyor + kicker running together. */
  public static Command conveyorAndKicker(ConveyorSubsystem conveyor, KickerSubsystem kicker) {
    return Commands.parallel(
        new RunConveyorCommand(conveyor, FuelPathConstants.kConveyorIn),
        new RunKickerCommand(kicker, FuelPathConstants.kKickerFeed)
    ).withName("ConveyorAndKicker");
  }

  /**
   * Full fuel path: run intake + conveyor + kicker while slowly retracting.
   * Pulls fuel inward by gradually retracting the linear intake.
   * On end: disables slow retract and retracts fully.
   */
  public static Command fullFuelPath(IntakeSubsystem intake, ConveyorSubsystem conveyor, KickerSubsystem kicker) {
    return Commands.parallel(
        new RunIntakeCommand(intake, FuelPathConstants.kIntakeInStandard),
        new RunConveyorCommand(conveyor, FuelPathConstants.kConveyorIn),
        new RunKickerCommand(kicker, FuelPathConstants.kKickerFeed)
    )
    .beforeStarting(() -> {
      intake.slowRetract(true);
    })
    .finallyDo(interrupted -> {
      intake.slowRetract(false);
      intake.pulse(false);
      intake.setIntakeDeployTarget(Setpoint.kRetracted);
    })
    .withName("FullFuelPath");
  }

  /**
   * Full fuel path with jam-protected kicker.
   * Slowly retracts intake while running intake + conveyor + kicker (jam detection).
   */
  public static Command fullFuelPathWithJamProtection(
      IntakeSubsystem intake, ConveyorSubsystem conveyor, KickerSubsystem kicker) {
    return Commands.parallel(
        new RunIntakeCommand(intake, FuelPathConstants.kIntakeInStandard),
        new RunConveyorCommand(conveyor, FuelPathConstants.kConveyorIn),
        new KickerJamProtectionCommand(kicker, FuelPathConstants.kKickerFeed)
    )
    .beforeStarting(() -> {
      intake.slowRetract(true);
    })
    .finallyDo(interrupted -> {
      intake.slowRetract(false);
      intake.pulse(false);
      intake.setIntakeDeployTarget(Setpoint.kRetracted);
    })
    .withName("FullFuelPathJamProtected");
  }

  // ── Intake with reverse pulse ──────────────────────────────────────

  /**
   * Runs intake roller forward, but when deploy position drops to ≤ 10 rotations
   * (~3/4 retracted), pulses reverse twice to dislodge stuck fuel, then resumes forward.
   */
  public static Command intakeWithReversePulse(IntakeSubsystem intake, double forwardSpeed) {
    return Commands.sequence(
        // Phase A: run forward until position crosses threshold
        Commands.run(() -> intake.setIntakeDutyCycle(forwardSpeed), intake)
            .until(() -> intake.getIntakePosition() <= 10),
        // Phase B: two reverse pulse cycles to dislodge stuck fuel
        Commands.runOnce(() -> intake.setIntakeDutyCycle(-3000), intake),
        Commands.waitSeconds(0.15),
        Commands.runOnce(() -> intake.setIntakeDutyCycle(forwardSpeed), intake),
        Commands.waitSeconds(0.3),
        Commands.runOnce(() -> intake.setIntakeDutyCycle(-3000), intake),
        Commands.waitSeconds(0.15),
        Commands.runOnce(() -> intake.setIntakeDutyCycle(forwardSpeed), intake),
        Commands.waitSeconds(0.3),
        // Phase C: resume forward for remainder
        Commands.run(() -> intake.setIntakeDutyCycle(forwardSpeed), intake)
    ).finallyDo(interrupted -> intake.setIntakeDutyCycle(0))
     .withName("IntakeWithReversePulse");
  }

  // ── Specialty ───────────────────────────────────────────────────────

  /** Jog conveyor forward then reverse briefly to clear a minor blockage. Runs once. */
  public static Command conveyorJog(ConveyorSubsystem conveyor) {
    return Commands.sequence(
        Commands.runOnce(() -> conveyor.setConveyorDutyCycle(FuelPathConstants.kConveyorJogSpeed), conveyor),
        Commands.waitSeconds(FuelPathConstants.kConveyorJogDurationSec),
        Commands.runOnce(() -> conveyor.setConveyorDutyCycle(-FuelPathConstants.kConveyorJogSpeed), conveyor),
        Commands.waitSeconds(FuelPathConstants.kConveyorJogDurationSec),
        Commands.runOnce(() -> conveyor.setConveyorDutyCycle(0), conveyor)
    ).finallyDo(interrupted -> { if (interrupted) conveyor.setConveyorDutyCycle(0); })
     .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
     .withName("ConveyorJog");
  }

  /**
   * Shuttle pulse: moves intake to shuttle center and enables in/out oscillation.
   * Runs intake roller while shuttling to agitate remaining fuel into conveyor.
   * Runs while held.
   */
  public static Command intakeShuttlePulse(IntakeSubsystem intake) {
    return Commands.run(() -> intake.setIntakeDutyCycle(FuelPathConstants.kIntakeInStandard), intake)
        .beforeStarting(() -> {
          intake.setIntakeDeployTarget(Setpoint.kShuttle);
          intake.pulse(true);
        })
        .finallyDo(interrupted -> {
          intake.setIntakeDutyCycle(0);
          intake.pulse(false);
        })
        .withName("IntakeShuttlePulse");
  }

  /** Brief kicker burst for single-ball feeding. Runs once. */
  public static Command kickerPulse(KickerSubsystem kicker) {
    return Commands.sequence(
        Commands.runOnce(() -> kicker.setKickerDutyCycle(FuelPathConstants.kKickerPulseSpeed), kicker),
        Commands.waitSeconds(FuelPathConstants.kKickerPulseDurationSec),
        Commands.runOnce(() -> kicker.setKickerDutyCycle(0), kicker)
    ).finallyDo(interrupted -> { if (interrupted) kicker.setKickerDutyCycle(0); })
     .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
     .withName("KickerPulse");
  }

  /**
   * Metered feed: conveyor runs always, kicker only runs when the shooter is at target RPM.
   * This is the key command for accurate shooting — prevents feeding before the flywheel
   * is ready.
   */
  public static Command meteredFeed(
      ConveyorSubsystem conveyor, KickerSubsystem kicker, ShooterSubsystem shooter) {
    // Track gating state so we only log on transitions, not every cycle
    boolean[] feeding = {false};
    return Commands.parallel(
        new RunConveyorCommand(conveyor, FuelPathConstants.kConveyorIn),
        Commands.run(
            () -> {
              boolean ready = shooter.isShooterReady();
              if (ready) {
                kicker.setKickerDutyCycle(FuelPathConstants.kKickerFeed);
                if (!feeding[0]) {
                  RobotLogger.log("METERED FEED: shooter ready, kicker feeding");
                  feeding[0] = true;
                }
              } else {
                kicker.setKickerDutyCycle(0);
                if (feeding[0]) {
                  RobotLogger.log("METERED FEED: shooter not ready, kicker gated");
                  feeding[0] = false;
                }
              }
            }, kicker
        ).finallyDo(interrupted -> kicker.setKickerDutyCycle(0))
    ).beforeStarting(() -> feeding[0] = false)
     .withName("MeteredFeed");
  }

  /** Instantly stops all fuel path motors and retracts the intake. One-shot command. */
  public static Command stopAll(IntakeSubsystem intake, ConveyorSubsystem conveyor, KickerSubsystem kicker) {
    return Commands.parallel(
        Commands.runOnce(() -> {
            intake.setIntakeDutyCycle(0);
            intake.pulse(false);
            intake.slowRetract(false);
            intake.setIntakeDeployTarget(Setpoint.kRetracted);
        }, intake),
        Commands.runOnce(() -> conveyor.setConveyorDutyCycle(0), conveyor),
        Commands.runOnce(() -> kicker.setKickerDutyCycle(0), kicker)
    ).withName("StopFuelPath");
  }

  /** Reverse everything at high speed. For clearing jams or ejecting fuel. */
  public static Command emergencyReverseAll(
      IntakeSubsystem intake, ConveyorSubsystem conveyor, KickerSubsystem kicker) {
    return Commands.parallel(
        new RunIntakeCommand(intake, FuelPathConstants.kEmergencyIntakeSpeed),
        new RunConveyorCommand(conveyor, FuelPathConstants.kEmergencyConveyorSpeed),
        new RunKickerCommand(kicker, FuelPathConstants.kEmergencyKickerSpeed)
    ).withName("EmergencyReverse");
  }

  /**
   * Shoot-on-the-move: intake extended + full pipeline + metered kicker + moving shooter map.
   * Driver keeps full translation; heading auto-locks to hub (handled in RobotContainer).
   */
  public static Command shootOnMove(
      IntakeSubsystem intake, ConveyorSubsystem conveyor,
      KickerSubsystem kicker, ShooterSubsystem shooter) {
    return Commands.parallel(
        new RunIntakeCommand(intake, FuelPathConstants.kIntakeInFast),
        new RunConveyorCommand(conveyor, FuelPathConstants.kConveyorIn),
        // Metered kicker: only feed when shooter is at speed
        Commands.run(() -> {
          if (shooter.isShooterReady()) {
            kicker.setKickerDutyCycle(FuelPathConstants.kKickerFeed);
          } else {
            kicker.setKickerDutyCycle(0);
          }
        }, kicker).finallyDo(i -> kicker.setKickerDutyCycle(0)),
        new ShootOnMoveCommand(shooter)
    )
    .beforeStarting(() -> intake.setIntakeDeployTarget(Setpoint.kExtended))
    .finallyDo(interrupted -> intake.setIntakeDeployTarget(Setpoint.kRetracted))
    .withName("ShootOnMove");
  }

  // ── Experimental feed methods (operator buttons 1-4) ───────────────

  /**
   * Experiment 1: "Brute Force" — max speed everything, straight retract, no oscillation.
   */
  public static Command experimentBruteForce(
      IntakeSubsystem intake, ConveyorSubsystem conveyor, KickerSubsystem kicker) {
    return Commands.parallel(
        intakeWithReversePulse(intake, 6000),
        new RunConveyorCommand(conveyor, 2500),
        new RunKickerCommand(kicker, 5000)
    )
    .beforeStarting(() -> intake.slowRetract(true))
    .finallyDo(interrupted -> {
      intake.slowRetract(false);
      intake.setIntakeDeployTarget(Setpoint.kRetracted);
    })
    .withName("Exp1-BruteForce");
  }

  /**
   * Experiment 2: "Oscillate From Start" — pulse + slow retract from the beginning.
   */
  public static Command experimentOscillateFromStart(
      IntakeSubsystem intake, ConveyorSubsystem conveyor, KickerSubsystem kicker) {
    return Commands.parallel(
        intakeWithReversePulse(intake, 4000),
        new RunConveyorCommand(conveyor, 2000),
        new RunKickerCommand(kicker, 5000)
    )
    .beforeStarting(() -> {
      intake.pulse(true);
      intake.slowRetract(true);
    })
    .finallyDo(interrupted -> {
      intake.pulse(false);
      intake.slowRetract(false);
      intake.setIntakeDeployTarget(Setpoint.kRetracted);
    })
    .withName("Exp2-OscillateFromStart");
  }

  /**
   * Experiment 3: "Staged Ramp" — gentle start, ramp up speeds in 3 phases.
   */
  public static Command experimentStagedRamp(
      IntakeSubsystem intake, ConveyorSubsystem conveyor, KickerSubsystem kicker) {
    return Commands.sequence(
        // Phase 1: gentle (1.5s)
        Commands.parallel(
            new RunIntakeCommand(intake, 2000),
            new RunConveyorCommand(conveyor, 1500),
            new RunKickerCommand(kicker, 3000)
        ).withTimeout(1.5),
        // Phase 2: medium (1.5s)
        Commands.parallel(
            new RunIntakeCommand(intake, 4000),
            new RunConveyorCommand(conveyor, 2000),
            new RunKickerCommand(kicker, 4000)
        ).withTimeout(1.5),
        // Phase 3: full speed + shuttle pulse + reverse pulses (hold indefinitely)
        Commands.parallel(
            intakeWithReversePulse(intake, 6000),
            new RunConveyorCommand(conveyor, 2500),
            new RunKickerCommand(kicker, 5000),
            Commands.runOnce(() -> {
              intake.setIntakeDeployTarget(Setpoint.kShuttle);
              intake.pulse(true);
            })
        )
    )
    .beforeStarting(() -> intake.slowRetract(true))
    .finallyDo(interrupted -> {
      intake.slowRetract(false);
      intake.pulse(false);
      intake.setIntakeDeployTarget(Setpoint.kRetracted);
    })
    .withName("Exp3-StagedRamp");
  }

  /**
   * Experiment 4: "Conveyor Jog Burst" — forward with periodic brief reversals.
   */
  public static Command experimentConveyorJog(
      IntakeSubsystem intake, ConveyorSubsystem conveyor, KickerSubsystem kicker) {
    return Commands.parallel(
        intakeWithReversePulse(intake, 4000),
        new RunKickerCommand(kicker, 5000),
        // Conveyor: cycle forward/reverse
        Commands.repeatingSequence(
            Commands.runOnce(() -> conveyor.setConveyorDutyCycle(2000), conveyor),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> conveyor.setConveyorDutyCycle(-500), conveyor),
            Commands.waitSeconds(0.1)
        )
    )
    .beforeStarting(() -> intake.slowRetract(true))
    .finallyDo(interrupted -> {
      intake.slowRetract(false);
      intake.setIntakeDeployTarget(Setpoint.kRetracted);
      conveyor.setConveyorDutyCycle(0);
    })
    .withName("Exp4-ConveyorJog");
  }

  /** Wraps KickerJamProtectionCommand for use as a standalone. */
  public static Command kickerWithJamProtection(KickerSubsystem kicker, double speed) {
    return new KickerJamProtectionCommand(kicker, speed);
  }

  /**
   * Feed until the beam break detects fuel staged at the kicker.
   * Runs conveyor + kicker until KickerSubsystem.hasFuel() returns true.
   */
  // public static Command feedUntilLoaded(ConveyorSubsystem conveyor, KickerSubsystem kicker) {
  //   return Commands.parallel(
  //       new RunConveyorCommand(conveyor, FuelPathConstants.kConveyorIn),
  //       new RunKickerCommand(kicker, FuelPathConstants.kKickerFeed)
  //   ).until(() -> kicker.hasFuel())
  //    .withName("FeedUntilLoaded");
  // }
}
