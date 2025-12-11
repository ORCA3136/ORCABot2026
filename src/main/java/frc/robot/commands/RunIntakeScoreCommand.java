// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Setpoint;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants;

// Import of shame
// import com.google.flatbuffers.Constants;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RunIntakeScoreCommand extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final LEDSubsystem led;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunIntakeScoreCommand(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem, LEDSubsystem ledSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    led = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Setpoint currentSetpoint = elevatorSubsystem.getSetpoint();

    switch (currentSetpoint) {
      case kFeederStation:
        intakeSubsystem.setIntakePower(Constants.IntakeConstants.IntakePowerLevels.kIn);
        break;
      case kLevel2:
        intakeSubsystem.setIntakePower(Constants.IntakeConstants.IntakePowerLevels.kOut);
        break;
      case kLevel3:
        intakeSubsystem.setIntakePower(Constants.IntakeConstants.IntakePowerLevels.kOut);
        break;
      case kLevel4:
        intakeSubsystem.setIntakePower(Constants.IntakeConstants.IntakePowerLevels.kOut);
        break;
      case kProcessor:
        intakeSubsystem.setIntakePower(Constants.IntakeConstants.IntakePowerLevels.kOut);
        break;
      case kTopAlgae:
        intakeSubsystem.setIntakePower(Constants.IntakeConstants.IntakePowerLevels.kIn);
        break;
      case kBottomAlgae:
        intakeSubsystem.setIntakePower(Constants.IntakeConstants.IntakePowerLevels.kIn);
        break;
      default:
        break;
    }
   
    led.ChangeLedColor(6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakePower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
