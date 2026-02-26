// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Runs the climber at a fixed speed until a position limit is reached.
 * Stops automatically if the climber exceeds max or min position (safety limit).
 */
public class RunClimberCommand extends Command {
  private final ClimberSubsystem m_climberSubsystem;
  private double climberPosition;
  private final double velocity;

  /**
   * @param subsystem The climber subsystem
   * @param inputVelocity RPM-scale speed (positive = extend, negative = retract)
   */
  public RunClimberCommand(ClimberSubsystem subsystem, double inputVelocity) {
    m_climberSubsystem = subsystem;
    velocity = inputVelocity;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climberSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climberPosition = m_climberSubsystem.getClimberPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Motor is commanded every cycle so it recovers automatically from CAN bus glitches.
  @Override
  public void execute() {
    climberPosition = m_climberSubsystem.getClimberPosition();
    m_climberSubsystem.setClimberDutyCycle(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.setClimberDutyCycle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (climberPosition > ClimberConstants.kClimberMaxPosition && velocity > 0)
      return true;
    if (climberPosition < ClimberConstants.kClimberMinPosition && velocity < 0)
      return true;
    return false;
  }
}
