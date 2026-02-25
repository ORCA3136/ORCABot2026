package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FuelPathConstants;
import frc.robot.RobotLogger;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Static factory methods for all fuel-path commands.
 *
 * One file = one "menu" of everything the intake -> conveyor -> kicker path can do.
 * Existing command classes (RunIntakeCommand, RunConveyorCommand, etc.) are the building
 * blocks these factories reuse.
 */
public final class FuelPathCommands {

  private FuelPathCommands() {} // Prevent instantiation

  // ── Individual: Intake ──────────────────────────────────────────────

  public static Command intakeIn(IntakeSubsystem intake) {
    return new RunIntakeCommand(intake, FuelPathConstants.kIntakeInStandard);
  }

  public static Command intakeInSlow(IntakeSubsystem intake) {
    return new RunIntakeCommand(intake, FuelPathConstants.kIntakeInSlow);
  }

  public static Command intakeInFast(IntakeSubsystem intake) {
    return new RunIntakeCommand(intake, FuelPathConstants.kIntakeInFast);
  }

  public static Command intakeOut(IntakeSubsystem intake) {
    return new RunIntakeCommand(intake, FuelPathConstants.kIntakeOutStandard);
  }

  public static Command intakeOutSlow(IntakeSubsystem intake) {
    return new RunIntakeCommand(intake, FuelPathConstants.kIntakeOutSlow);
  }

  public static Command intakeOutFast(IntakeSubsystem intake) {
    return new RunIntakeCommand(intake, FuelPathConstants.kIntakeOutFast);
  }

  // ── Individual: Conveyor ────────────────────────────────────────────

  public static Command conveyorIn(ConveyorSubsystem conveyor) {
    return new RunConveyorCommand(conveyor, FuelPathConstants.kConveyorIn);
  }

  public static Command conveyorOut(ConveyorSubsystem conveyor) {
    return new RunConveyorCommand(conveyor, FuelPathConstants.kConveyorOut);
  }

  // ── Individual: Kicker ──────────────────────────────────────────────

  public static Command kickerFeed(KickerSubsystem kicker) {
    return new RunKickerCommand(kicker, FuelPathConstants.kKickerFeed);
  }

  public static Command kickerOut(KickerSubsystem kicker) {
    return new RunKickerCommand(kicker, FuelPathConstants.kKickerOut);
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
   * Full fuel path: deploy intake, enable oscillation, run intake + conveyor + kicker.
   * Stops oscillation and retracts on end.
   */
  public static Command fullFuelPath(IntakeSubsystem intake, ConveyorSubsystem conveyor, KickerSubsystem kicker) {
    return Commands.parallel(
        new RunIntakeCommand(intake, FuelPathConstants.kIntakeInStandard),
        new RunConveyorCommand(conveyor, FuelPathConstants.kConveyorIn),
        new RunKickerCommand(kicker, FuelPathConstants.kKickerFeed)
    )
    .beforeStarting(() -> {
      intake.deployIntake(true);
      intake.ocillateIntake(true);
    })
    .finallyDo(interrupted -> {
      intake.ocillateIntake(false);
    })
    .withName("FullFuelPath");
  }

  /**
   * Full fuel path with jam-protected kicker.
   * Deploy intake, enable oscillation, run intake + conveyor + kicker (jam detection).
   */
  public static Command fullFuelPathWithJamProtection(
      IntakeSubsystem intake, ConveyorSubsystem conveyor, KickerSubsystem kicker) {
    return Commands.parallel(
        new RunIntakeCommand(intake, FuelPathConstants.kIntakeInStandard),
        new RunConveyorCommand(conveyor, FuelPathConstants.kConveyorIn),
        new KickerJamProtectionCommand(kicker, FuelPathConstants.kKickerFeed)
    )
    .beforeStarting(() -> {
      intake.deployIntake(true);
      intake.ocillateIntake(true);
    })
    .finallyDo(interrupted -> {
      intake.ocillateIntake(false);
    })
    .withName("FullFuelPathJamProtected");
  }

  // ── Specialty ───────────────────────────────────────────────────────

  /** Jog conveyor forward then reverse briefly to clear a minor blockage. Runs once. */
  public static Command conveyorJog(ConveyorSubsystem conveyor) {
    return Commands.sequence(
        Commands.runOnce(() -> conveyor.setConveyorVelocity(FuelPathConstants.kConveyorJogSpeed)),
        Commands.waitSeconds(FuelPathConstants.kConveyorJogDurationSec),
        Commands.runOnce(() -> conveyor.setConveyorVelocity(-FuelPathConstants.kConveyorJogSpeed)),
        Commands.waitSeconds(FuelPathConstants.kConveyorJogDurationSec),
        Commands.runOnce(() -> conveyor.setConveyorVelocity(0))
    ).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
     .withName("ConveyorJog");
  }

  /** Brief kicker burst for single-ball feeding. Runs once. */
  public static Command kickerPulse(KickerSubsystem kicker) {
    return Commands.sequence(
        Commands.runOnce(() -> kicker.setKickerVelocity(FuelPathConstants.kKickerPulseSpeed)),
        Commands.waitSeconds(FuelPathConstants.kKickerPulseDurationSec),
        Commands.runOnce(() -> kicker.setKickerVelocity(0))
    ).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
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
                kicker.setKickerVelocity(FuelPathConstants.kKickerFeed);
                if (!feeding[0]) {
                  RobotLogger.log("METERED FEED: shooter ready, kicker feeding");
                  feeding[0] = true;
                }
              } else {
                kicker.setKickerVelocity(0);
                if (feeding[0]) {
                  RobotLogger.log("METERED FEED: shooter not ready, kicker gated");
                  feeding[0] = false;
                }
              }
            }, kicker
        ).finallyDo(interrupted -> kicker.setKickerVelocity(0))
    ).withName("MeteredFeed");
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

  /** Wraps KickerJamProtectionCommand for use as a standalone. */
  public static Command kickerWithJamProtection(KickerSubsystem kicker, double speed) {
    return new KickerJamProtectionCommand(kicker, speed);
  }

  /**
   * Feed until loaded. Currently runs conveyor + kicker continuously.
   * TODO: Stop when beam break sensor triggers (sensor not yet wired).
   */
  public static Command feedUntilLoaded(ConveyorSubsystem conveyor, KickerSubsystem kicker) {
    return Commands.parallel(
        new RunConveyorCommand(conveyor, FuelPathConstants.kConveyorIn),
        new RunKickerCommand(kicker, FuelPathConstants.kKickerFeed)
    ).withName("FeedUntilLoaded");
  }
}
