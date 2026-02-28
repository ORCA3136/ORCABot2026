// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;


/**
 * Slowly oscillates the hood between LOWER_BOUND and UPPER_BOUND (in encoder position units).
 * The hood reverses direction when it reaches either limit.
 * Used for testing and calibration via the D-pad.
 */
public class SlowHoodMove extends Command {
  private final ShooterSubsystem m_shooterSubsystem;

  // TODO: TUNE ON ROBOT — verify degrees per second feels right on the actual robot
  private static final double DEGREES_PER_SECOND = 4;
  /** Converts degrees/sec to encoder position units/sec so we can add it to the hood target. */
  private static final double ENCODER_UNITS_PER_DEGREE =
      (1.0 / 360.0) * HoodConstants.kEncoderGearRatio * HoodConstants.kMotorGearRatio;

  private static final double UPPER_BOUND = 5;  // max target (encoder position units)
  private static final double LOWER_BOUND = 1;  // min target (encoder position units)

  double currentTime;
  double dTime; // Delta time
  double targetPosition; // Rotations

  /** @param shooterSubsystem The hood subsystem to oscillate */
  public SlowHoodMove(ShooterSubsystem shooterSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentTime = Timer.getTimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetPosition = m_shooterSubsystem.getHoodTarget();
    if ((targetPosition >= UPPER_BOUND && m_shooterSubsystem.getHoodMovingForward()) || (targetPosition <= LOWER_BOUND && !m_shooterSubsystem.getHoodMovingForward())) {
      m_shooterSubsystem.changeHoodDirection();
    }
    dTime = Timer.getTimestamp() - currentTime;
    targetPosition += DEGREES_PER_SECOND * ENCODER_UNITS_PER_DEGREE * dTime * (m_shooterSubsystem.getHoodMovingForward() ? 1 : -1);
    m_shooterSubsystem.setHoodTarget(targetPosition);
    currentTime = Timer.getTimestamp();
  }

  // Called once the command ends or is interrupted.
  // Intentionally does not reset the hood target — the PID holds the last position.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
