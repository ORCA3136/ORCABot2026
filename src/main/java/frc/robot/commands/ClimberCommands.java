package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
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
    Command cmd = Commands.sequence(
        // Step 1: stow intake
        Commands.runOnce(() -> intake.setIntakeDeployTarget(IntakeSubsystem.Setpoint.kUp)),

        // Step 2: wait for intake to reach stowed position (with timeout safety)
        Commands.waitUntil(() ->
            Math.abs(intake.getIntakeDeployPosition() - IntakeConstants.kMaxDeployPosition) < 0.02
        ).withTimeout(1.5),

        // Step 3: move arm to horizontal and hold indefinitely
        Commands.run(() -> climber.setArmPosition(climber.getHorizontalDegrees()), climber)
    );
    cmd.addRequirements(intake);  // climber already required by Step 3
    return cmd;
  }

  /**
   * Execute the climb: pull arm to climbed position and hold.
   * Does NOT cut power on end — robot is hanging, brake mode + 318:1 reduction holds it.
   * Guard: will not execute unless arm is near horizontal (deployed via prepareToClimb).
   *
   * @param climber The climber subsystem
   */
  public static Command executeClimb(ClimberSubsystem climber) {
    return Commands.sequence(
        // Guard: don't pull unless arm is near horizontal
        Commands.waitUntil(() -> climber.getArmDegrees() > climber.getHorizontalDegrees() - 15)
            .withTimeout(0.1),

        // Step 1: pull up to climbed position (with timeout safety)
        Commands.run(() -> climber.setArmPosition(climber.getClimbedDegrees()), climber)
            .until(() -> climber.atSetpoint(5.0))
            .withTimeout(5.0),

        // Step 2: hold at climbed position forever
        Commands.run(() -> climber.setArmPosition(climber.getClimbedDegrees()), climber)
    ).finallyDo((interrupted) -> {
      DataLogManager.log("ClimberCommands: executeClimb ended, interrupted=" + interrupted);
    });
  }
}
