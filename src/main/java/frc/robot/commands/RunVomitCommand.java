// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RunVomitCommand extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final double powerSetPoint;
  //private LaserCan lidarObect;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunVomitCommand(IntakeSubsystem intakeSubsystem, double power) {
    this.intakeSubsystem = intakeSubsystem;
    powerSetPoint = power;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.setIntakePower(powerSetPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakePower(0);
    // isFinished();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
