package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command factories for the climbing sequence.
 *
 * Climb is a two-button operation:
 * 1. Prepare: stow intake for clearance, then move arm to horizontal and hold.
 * 2. Execute: pull arm to climbed position and hold forever (robot is hanging).
 */
public final class ClimberCommands {

  private ClimberCommands() {} // static factory only

  /**
   * Prepare to climb: stow the intake, wait for clearance, then move arm to horizontal.
   * Holds at horizontal until cancelled.
   *
   * @param climber The climber subsystem
   * @param intake  The intake subsystem (stowed for arm clearance)
   */
  public static Command prepareToClimb(ClimberSubsystem climber, IntakeSubsystem intake) {
    return Commands.sequence(
        // Step 1: stow intake
        Commands.runOnce(() -> intake.setIntakeDeployTarget(IntakeSubsystem.Setpoint.kUp)),

        // Step 2: wait for intake to reach stowed position (with timeout safety)
        Commands.waitUntil(() ->
            Math.abs(intake.getIntakeDeployPosition() - IntakeConstants.kMaxDeployPosition) < 0.02
        ).withTimeout(1.5),

        // Step 3: move arm to horizontal and hold indefinitely
        Commands.run(() -> climber.setArmPosition(climber.getHorizontalDegrees()), climber)
    );
  }

  /**
   * Execute the climb: pull arm to climbed position and hold.
   * Does NOT cut power on end — robot is hanging, brake mode + 318:1 reduction holds it.
   *
   * @param climber The climber subsystem
   */
  public static Command executeClimb(ClimberSubsystem climber) {
    return Commands.sequence(
        // Step 1: pull up to climbed position (with timeout safety)
        Commands.run(() -> climber.setArmPosition(climber.getClimbedDegrees()), climber)
            .until(() -> climber.atSetpoint(5.0))
            .withTimeout(5.0),

        // Step 2: hold at climbed position forever
        Commands.run(() -> climber.setArmPosition(climber.getClimbedDegrees()), climber)
    ).finallyDo((interrupted) -> {
      // Log but do NOT stop — robot is hanging, brake mode holds
      System.out.println("ClimberCommands: executeClimb ended, interrupted=" + interrupted);
    });
  }
}
