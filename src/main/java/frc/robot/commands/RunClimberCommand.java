// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RunClimberCommand extends Command {
  private final ClimberSubsystem climberSubsystem;
  private final double powerSetPoint;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunClimberCommand(ClimberSubsystem climberSubsystem, double power) {
    this.climberSubsystem = climberSubsystem;
    powerSetPoint = power;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climberSubsystem.setClimbingMode(true);
    climberSubsystem.setClimberPower(powerSetPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  climberSubsystem.setClimberPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (climberSubsystem.getClimberPosition() > Constants.ClimberConstants.kClimberOutPos && powerSetPoint > 0)
     || (climberSubsystem.getClimberPosition() < Constants.ClimberConstants.kClimberInPos && powerSetPoint < 0);
  }
}
