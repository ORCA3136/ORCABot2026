// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** A command that uses a Climber subsystem. */
public class RunClimbSequenceCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem m_climber;
  private final ElevatorSubsystem m_elevator;
  private boolean flipped;
  private boolean isOut;

  /**
   * Creates a new ClimberCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunClimbSequenceCommand(ClimberSubsystem subsystem, ElevatorSubsystem elevator, boolean isOut) {
    m_climber = subsystem;
    m_elevator = elevator;
    this.isOut = isOut;
    addRequirements(subsystem, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.setClimbingMode(true);

    if (!m_climber.isFlipped()) {
      m_climber.setFunnelPower(Constants.ClimberConstants.kFunnelSpeed);
    } else {
      m_climber.setFunnelPower(0);
    }

    m_elevator.setWristTarget(Constants.WristConstants.WristSetpoints.kClimb);

    if (isOut) {
      m_climber.setClimberPower(Constants.ClimberConstants.kClimberInSpeed);
    } else {
      m_climber.setClimberPower(Constants.ClimberConstants.kClimberOutSpeed);
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_climber.isFlipped()) {
      m_climber.setFunnelPower(0);
    }

    if (isOut) {
      if (m_climber.getClimberPosition() < Constants.ClimberConstants.kClimberInPos) {
        m_climber.setClimberPower(0);
      } else {
        m_climber.setClimberPower(Constants.ClimberConstants.kClimberInSpeed);
      }
    }
    else {
      if (m_climber.getClimberPosition() > Constants.ClimberConstants.kClimberOutPos) {
        m_climber.setClimberPower(0);
      } else {
        m_climber.setClimberPower(Constants.ClimberConstants.kClimberOutSpeed);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.setClimberPower(0);
    m_climber.setFunnelPower (0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climber.isFlipped() 
    && 
    ((m_climber.getClimberPosition() > Constants.ClimberConstants.kClimberOutPos && !isOut)
    ||
    (m_climber.getClimberPosition() < Constants.ClimberConstants.kClimberInPos && isOut));
  }
}
