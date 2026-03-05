// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Runs the climber at a fixed speed with software position limits.
 * Stops automatically if the arm exceeds max or min degrees.
 *
 * @deprecated Use inline Commands.runEnd() with setManualDutyCycle() instead,
 *             or use ClimberCommands for the sequenced climb.
 */
@Deprecated
public class RunClimberCommand extends Command {
  private final ClimberSubsystem m_climberSubsystem;
  private final double velocity;

  /**
   * @param subsystem The climber subsystem
   * @param inputVelocity RPM-scale speed (positive = extend, negative = retract)
   */
  public RunClimberCommand(ClimberSubsystem subsystem, double inputVelocity) {
    m_climberSubsystem = subsystem;
    velocity = inputVelocity;
    addRequirements(m_climberSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_climberSubsystem.setManualDutyCycle(velocity);
  }

  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.stopManual();
  }

  @Override
  public boolean isFinished() {
    double armDeg = m_climberSubsystem.getArmDegrees();
    if (armDeg > ClimberConstants.kMaxArmDegrees && velocity > 0) return true;
    if (armDeg < ClimberConstants.kMinArmDegrees && velocity < 0) return true;
    return false;
  }
}
