// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** Runs the hood motor at a fixed duty cycle speed. Stops when the command ends. */
public class RunHoodCommand extends Command {
  private final ShooterSubsystem m_shooterSubsystem;
  private final double velocity;

  /**
   * @param shooterSubsystem The hood subsystem
   * @param velocity RPM-scale speed for manual hood control
   */
  public RunHoodCommand(ShooterSubsystem shooterSubsystem, double velocity) {
    m_shooterSubsystem = shooterSubsystem;

    this.velocity = velocity;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  // Motor is commanded every cycle so it recovers automatically from CAN bus glitches.
  @Override
  public void execute() {
    m_shooterSubsystem.setHoodDutyCycle(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setHoodDutyCycle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
