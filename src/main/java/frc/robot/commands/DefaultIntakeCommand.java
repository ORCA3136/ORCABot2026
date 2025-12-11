// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Configs.ClimberConfigs;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Setpoint;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DefaultIntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private IntakeSubsystem intakeSubsystem;
  private ClimberSubsystem climberSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  //private LaserCan lidarObect;
  private VisionSubsystem vision;
  // private LEDSubsystem led;

  private boolean pulse = false;
  private double time;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem, VisionSubsystem vision, 
                              ClimberSubsystem climberSubsystem, ElevatorSubsystem elevatorSubsystem) {
    this.climberSubsystem = climberSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.vision = vision;
    // led = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = Timer.getTimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climberSubsystem.getClimbingMode()) {
      intakeSubsystem.setIntakePower(0);
    } 
    
    else {

      if (elevatorSubsystem.getSetpoint() == Setpoint.kTopAlgae || elevatorSubsystem.getSetpoint() == Setpoint.kBottomAlgae) {
        climberSubsystem.setFunnelPower(0);
        time = Timer.getTimestamp();
        
        if (elevatorSubsystem.hasAlgae()) {
          intakeSubsystem.setIntakePower(Constants.IntakeConstants.IntakePowerLevels.kAlgaeHold);
        }
        else {
          intakeSubsystem.setIntakePower(Constants.IntakeConstants.IntakePowerLevels.kAlgaeIn);
        }
      }

      else {
        if (vision.hasCoralInIntake()) {
          intakeSubsystem.setIntakePower(0.08);
          climberSubsystem.setFunnelPower(0);
          time = Timer.getTimestamp();
        } else if (vision.hasCoralInFunnel()) {
          intakeSubsystem.setIntakePower(-0.5);
        } else {
          intakeSubsystem.setIntakePower(0);
          climberSubsystem.setFunnelPower(0);
          time = Timer.getTimestamp();
        } 

        if (!pulse && Timer.getTimestamp() > time + 0.175) {
          climberSubsystem.setFunnelPower(-0.30);
          time = Timer.getTimestamp();
          pulse = !pulse;
        } else if (pulse && Timer.getTimestamp() > time + 0.075) {
          climberSubsystem.setFunnelPower(0.1);
          time = Timer.getTimestamp();
          pulse = !pulse;
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakePower(0);
    if (!climberSubsystem.getClimbingMode()) {
      climberSubsystem.setFunnelPower(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
