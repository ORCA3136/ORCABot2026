// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** Sets the shooter flywheel to a target RPM. Stops the flywheel when the command ends. */
public class FixedShootCommand extends Command {
  private final ShooterSubsystem m_shooterSubsystem;
  private double distance;

  /** @param shooterSubsystem The shooter subsystem */
  public FixedShootCommand(ShooterSubsystem shooterSubsystem, double distance) {
    m_shooterSubsystem = shooterSubsystem;
    this.distance = distance;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // set shooter map
    m_shooterSubsystem.setShooterMapOnly(distance);
  }

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
