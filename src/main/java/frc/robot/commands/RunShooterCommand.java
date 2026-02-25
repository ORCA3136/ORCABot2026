// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** Sets the shooter flywheel to a target RPM. Stops the flywheel when the command ends. */
public class RunShooterCommand extends Command {
  private final ShooterSubsystem m_shooterSubsystem;
  private final double velocity;

  /**
   * @param shooterSubsystem The shooter subsystem
   * @param inputVelocity Target RPM for the flywheel (ramping is handled by ShooterSubsystem)
   */
  public RunShooterCommand(ShooterSubsystem shooterSubsystem, double inputVelocity) {
    m_shooterSubsystem = shooterSubsystem;

    this.velocity = inputVelocity;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.setShooterVelocityTarget(velocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setShooterVelocityTarget(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
