// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.HoodSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;


/** An example command that uses an example subsystem. */
public class SlowHoodMove extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final HoodSubsystem m_hoodSubsystem;

  private final double slowHoodSpeed = 4; // Degrees per second (DPS)

  double currentTime;
  double dTime; // Delta time
  double targetPosition; // Rotations

  /**
   * Creates a new ExampleCommand.
   *
   * @param hoodSubsystem The subsystem used by this command.
   */
  public SlowHoodMove(HoodSubsystem hoodSubsystem) {
    m_hoodSubsystem = hoodSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hoodSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hoodSubsystem.getClass();
    currentTime = Timer.getTimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetPosition = m_hoodSubsystem.getHoodTarget();
    if (targetPosition >= 5 && m_hoodSubsystem.getHoodMovingForward() == true || targetPosition <= 1 && m_hoodSubsystem.getHoodMovingForward() == false) {
      m_hoodSubsystem.changeHoodDirection();
    }
    dTime = Timer.getTimestamp() - currentTime;
    targetPosition += slowHoodSpeed * dTime * (m_hoodSubsystem.getHoodMovingForward() ? 1 : -1);
    m_hoodSubsystem.setHoodTarget(targetPosition);
    currentTime = Timer.getTimestamp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
