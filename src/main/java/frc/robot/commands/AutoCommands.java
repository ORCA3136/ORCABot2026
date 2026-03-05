package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FuelPathConstants;
import frc.robot.RobotLogger;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Setpoint;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Static factory methods for auto-safe commands.
 *
 * Every command returned here:
 *   - Has a descriptive .withName() for DataLog tracing
 *   - Declares proper subsystem requirements
 *   - Has a timeout (where applicable) so it can never block an auto
 *   - Cleans up motors in finallyDo() so nothing is left spinning
 *
 * Register these in RobotContainer.configureNamedCommands() for PathPlanner use.
 */
public final class AutoCommands {

  private AutoCommands() {} // Prevent instantiation

  // ── Group A: Instant (runOnce) commands ──────────────────────────────

  /** Deploy intake to kDown position. Finishes instantly. */
  public static Command intakeDown(IntakeSubsystem intake) {
    return Commands.runOnce(() -> intake.setIntakeDeployTarget(Setpoint.kDown), intake)
        .withName("IntakeDown");
  }

  /** Move intake to kSafe position. Finishes instantly. */
  public static Command intakeSafe(IntakeSubsystem intake) {
    return Commands.runOnce(() -> intake.setIntakeDeployTarget(Setpoint.kSafe), intake)
        .withName("IntakeSafe");
  }

  /** Retract intake to kUp position. Finishes instantly. */
  public static Command intakeUp(IntakeSubsystem intake) {
    return Commands.runOnce(() -> intake.setIntakeDeployTarget(Setpoint.kUp), intake)
        .withName("IntakeUp");
  }

  /**
   * Spin up shooter from distance map (no feed).
   * Runs continuously — meant to be used in a parallel/race group.
   */
  public static Command spinUpFromDistance(ShooterSubsystem shooter, SwerveSubsystem swerve) {
    return Commands.run(() -> shooter.setShooterMap(), shooter)
        .withName("SpinUpFromDistance");
  }

  // ── Group B: Duration-based commands (timeout + cleanup) ─────────────

  /** Run intake rollers at given speed with timeout. Stops rollers on end. */
  public static Command runIntake(IntakeSubsystem intake, double speed, double timeoutSec) {
    return Commands.run(() -> intake.setIntakeDutyCycle(speed), intake)
        .withTimeout(timeoutSec)
        .finallyDo(interrupted -> intake.setIntakeDutyCycle(0))
        .withName("RunIntake" + (int) timeoutSec + "s");
  }

  /** Run conveyor at given speed with timeout. Stops conveyor on end. */
  public static Command runConveyor(ConveyorSubsystem conveyor, double speed, double timeoutSec) {
    return Commands.run(() -> conveyor.setConveyorDutyCycle(speed), conveyor)
        .withTimeout(timeoutSec)
        .finallyDo(interrupted -> conveyor.setConveyorDutyCycle(0))
        .withName("RunConveyor" + (int) timeoutSec + "s");
  }

  /** Run kicker at given speed with timeout. Stops kicker on end. */
  public static Command runKicker(KickerSubsystem kicker, double speed, double timeoutSec) {
    return Commands.run(() -> kicker.setKickerDutyCycle(speed), kicker)
        .withTimeout(timeoutSec)
        .finallyDo(interrupted -> kicker.setKickerDutyCycle(0))
        .withName("RunKicker" + (int) timeoutSec + "s");
  }

  /** Run conveyor + kicker together with timeout. Stops both on end. */
  public static Command feedAll(ConveyorSubsystem conveyor, KickerSubsystem kicker,
                                double conveyorSpeed, double kickerSpeed, double timeoutSec) {
    return Commands.run(() -> {
          conveyor.setConveyorDutyCycle(conveyorSpeed);
          kicker.setKickerDutyCycle(kickerSpeed);
        }, conveyor, kicker)
        .withTimeout(timeoutSec)
        .finallyDo(interrupted -> {
          conveyor.setConveyorDutyCycle(0);
          kicker.setKickerDutyCycle(0);
        })
        .withName("FeedAll" + (int) timeoutSec + "s");
  }

  /**
   * Full intake sequence: deploy intake to kDown, run rollers, then retract to kSafe on end.
   * Stops rollers and retracts when timeout expires or command is interrupted.
   */
  public static Command intakeWithDeploy(IntakeSubsystem intake, double speed, double timeoutSec) {
    return Commands.run(() -> intake.setIntakeDutyCycle(speed), intake)
        .beforeStarting(() -> intake.setIntakeDeployTarget(Setpoint.kDown))
        .withTimeout(timeoutSec)
        .finallyDo(interrupted -> {
          intake.setIntakeDutyCycle(0);
          intake.setIntakeDeployTarget(Setpoint.kSafe);
        })
        .withName("IntakeWithDeploy" + (int) timeoutSec + "s");
  }

  // ── Group C: Smart/composite commands ────────────────────────────────

  /**
   * Aim + spin up + feed: the real "shoot to hub" for auto.
   *
   * Phase 1 (40% of timeout): Spin up flywheel from distance map, wait until ready.
   * Phase 2 (60% of timeout): Keep updating shooter map; feed when ready, gate when not.
   * Cleanup: Stop all motors.
   */
  public static Command aimAndShoot(ShooterSubsystem shooter, SwerveSubsystem swerve,
                                     KickerSubsystem kicker, ConveyorSubsystem conveyor,
                                     double timeoutSec) {
    return Commands.sequence(
        // Phase 1: Spin up
        Commands.run(() -> shooter.setShooterMap(), shooter)
            .until(() -> shooter.isShooterReady())
            .withTimeout(timeoutSec * 0.4)
            .withName("AimAndShoot-SpinUp"),
        // Phase 2: Feed while maintaining shooter
        Commands.run(() -> {
              shooter.setShooterMap();
              if (shooter.isShooterReady()) {
                kicker.setKickerDutyCycle(FuelPathConstants.kKickerFeed);
                conveyor.setConveyorDutyCycle(FuelPathConstants.kConveyorIn);
              } else {
                kicker.setKickerDutyCycle(0);
                conveyor.setConveyorDutyCycle(0);
              }
            }, shooter, kicker, conveyor)
            .withTimeout(timeoutSec * 0.6)
            .withName("AimAndShoot-Feed")
    )
    .finallyDo(interrupted -> {
      kicker.setKickerDutyCycle(0);
      conveyor.setConveyorDutyCycle(0);
      shooter.setShooterVelocityTarget(0);
      shooter.setHoodTarget(0);
      RobotLogger.log("AimAndShoot: " + (interrupted ? "TIMED_OUT" : "COMPLETE"));
    })
    .withName("AimAndShoot");
  }

  /** Metered feed with timeout. Wraps existing FuelPathCommands.meteredFeed. */
  public static Command meteredFeedTimed(ConveyorSubsystem conveyor, KickerSubsystem kicker,
                                          ShooterSubsystem shooter, double timeoutSec) {
    return FuelPathCommands.meteredFeed(conveyor, kicker, shooter)
        .withTimeout(timeoutSec)
        .withName("MeteredFeed" + (int) timeoutSec + "s");
  }

  /**
   * Full auto cycle: deploy intake + run + retract, then aim + shoot.
   * For PathPlanner event markers that need to do everything in one command.
   */
  public static Command intakeAndShoot(IntakeSubsystem intake, ConveyorSubsystem conveyor,
                                        KickerSubsystem kicker, ShooterSubsystem shooter,
                                        SwerveSubsystem swerve,
                                        double intakeTimeSec, double shootTimeSec) {
    return Commands.sequence(
        intakeWithDeploy(intake, FuelPathConstants.kIntakeInStandard, intakeTimeSec),
        aimAndShoot(shooter, swerve, kicker, conveyor, shootTimeSec)
    ).withName("IntakeAndShoot");
  }
}
