// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** Sets the shooter flywheel to a target RPM. Stops the flywheel when the command ends. */
public class ShootCommand extends Command {
  private final ShooterSubsystem m_shooterSubsystem;
  private final SwerveSubsystem m_swerveSubsystem;

  private double distanceToHub;

  /**
   * @param shooterSubsystem The shooter subsystem
   * @param inputVelocity Target RPM for the flywheel (ramping is handled by ShooterSubsystem)
   */
  public ShootCommand(ShooterSubsystem shooterSubsystem, SwerveSubsystem swerveSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    m_swerveSubsystem = swerveSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get distance to hub
    distanceToHub = m_swerveSubsystem.getDistanceToHub();
    // set shooter and hood map
    m_shooterSubsystem.setShooterMap(distanceToHub);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setHoodTarget(0);
    m_shooterSubsystem.setShooterVelocityTarget(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
