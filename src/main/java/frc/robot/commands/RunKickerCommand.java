// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.KickerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** Runs the kicker wheel at a fixed speed. Stops when the command ends. */
public class RunKickerCommand extends Command {
  private final KickerSubsystem m_KickerSubsystem;
  private final double velocity;

  /**
   * @param kickerSubsystem The kicker subsystem
   * @param kickerVelocity RPM-scale speed (positive = feed into shooter, negative = reverse)
   */
  public RunKickerCommand(KickerSubsystem kickerSubsystem, double kickerVelocity) {
    m_KickerSubsystem = kickerSubsystem;
    velocity = kickerVelocity;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(kickerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  // Motor is commanded every cycle so it recovers automatically from CAN bus glitches.
  @Override
  public void execute() {
    m_KickerSubsystem.setKickerDutyCycle(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_KickerSubsystem.setKickerDutyCycle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
