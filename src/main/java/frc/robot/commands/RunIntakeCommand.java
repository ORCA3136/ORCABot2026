// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** Runs the intake roller at a fixed speed. Stops when the command ends. */
public class RunIntakeCommand extends Command {
  private final IntakeSubsystem m_intakeSubsystem;
  private final double intakeVelocity;

  /**
   * @param intakeSubsystem The intake subsystem
   * @param intakeVelocity RPM-scale speed (positive = intake fuel, negative = outtake)
   */
  public RunIntakeCommand(IntakeSubsystem intakeSubsystem, double intakeVelocity) {
    m_intakeSubsystem = intakeSubsystem;

    this.intakeVelocity = intakeVelocity;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  // Motor is commanded every cycle so it recovers automatically from CAN bus glitches.
  @Override
  public void execute() {
    m_intakeSubsystem.setIntakeDutyCycle(intakeVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setIntakeDutyCycle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
