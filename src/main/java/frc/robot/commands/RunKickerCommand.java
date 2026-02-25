// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RunKickerCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final KickerSubsystem m_KickerSubsystem;
  private final double velocity;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunKickerCommand(KickerSubsystem kickerSubsystem, double kickerVelocity) {
    m_KickerSubsystem = kickerSubsystem;
    velocity = kickerVelocity;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(kickerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    m_KickerSubsystem.setKickerVelocity(velocity);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_KickerSubsystem.setKickerVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
