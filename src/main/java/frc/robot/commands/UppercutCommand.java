// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;



/** An example command that uses an example subsystem. */
public class UppercutCommand extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private final IntakeSubsystem intake;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public UppercutCommand(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intake) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.intake = intake;
    
    addRequirements(elevatorSubsystem, intake);
  }
  // Constants.Limits.kElevatorMaxHeight

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.setTargetSetpoint(ElevatorSubsystem.Setpoint.kTop);
    intake.setIntakePower(Constants.IntakeConstants.IntakePowerLevels.kAlgaeHold); // (Constants.IntakeConstants.IntakePowerLevels.kAlgaeHold);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevatorSubsystem.getElevatorPosition() > 37) {
      elevatorSubsystem.setTargetSetpoint(ElevatorSubsystem.Setpoint.kBarge);
    }

    if (elevatorSubsystem.getWristPosition() < 125) {
      intake.setIntakePower(Constants.IntakeConstants.IntakePowerLevels.kAlgaeOut);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setTargetSetpoint(ElevatorSubsystem.Setpoint.kTop);
    intake.setIntakePower(Constants.IntakeConstants.IntakePowerLevels.kAlgaeHold);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (elevatorSubsystem.getElevatorPosition() >= Constants.ElevatorConstants.ElevatorSetpoints.kBarge - 1 && elevatorSubsystem.getWristPosition() < 30);
  }
}