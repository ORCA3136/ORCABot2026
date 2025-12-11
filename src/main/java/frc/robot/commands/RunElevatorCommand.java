// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;




/** An example command that uses an example subsystem. */
public class RunElevatorCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem elevatorSubsystem;
  private final double powerSetPoint;
  //private boolean safe = true;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunElevatorCommand(ElevatorSubsystem elevatorSubsystem, double power) {
    this.elevatorSubsystem = elevatorSubsystem;
    powerSetPoint = power;
    
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.setElevatorPower(powerSetPoint);
    elevatorSubsystem.setElevatorManuallyMoving(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setElevatorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (powerSetPoint < 0 && elevatorSubsystem.getElevatorPosition() < Constants.Limits.kElevatorMinHeight)
        || (powerSetPoint > 0 && elevatorSubsystem.getElevatorPosition() > Constants.Limits.kElevatorMaxHeight);
  }
}