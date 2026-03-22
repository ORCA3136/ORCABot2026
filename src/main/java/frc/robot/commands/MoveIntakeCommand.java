// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Setpoint;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;

/** Runs the intake roller at a fixed speed. Stops when the command ends. */
public class MoveIntakeCommand extends Command {
  private final IntakeSubsystem m_intakeSubsystem;
  private boolean reachedTarget = false;
  private double setpoint;
  private SparkClosedLoopController deployPIDController;

  /**
   * @param intakeSubsystem The intake subsystem
   * @param intakeTarget RPM-scale speed (positive = intake fuel, negative = outtake)
   */
  public MoveIntakeCommand(IntakeSubsystem intakeSubsystem, Setpoint intakeTarget) {
    m_intakeSubsystem = intakeSubsystem;
    deployPIDController = m_intakeSubsystem.getDeployClosedLoopController();

    switch (intakeTarget) {
      case kShuttle:
        setpoint = IntakeConstants.kShuttleCenter;
        break;
      case kExtended:
        setpoint = IntakeConstants.kExtendedPosition;
        break;
        // Default is retracted
      default:
        setpoint = IntakeConstants.kRetractedPosition;
        break;
    }
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    deployPIDController.setSetpoint(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Motor is commanded every cycle so it recovers automatically from CAN bus glitches.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (reachedTarget) return true;

    // if (setpoint < 1 && )

    return false;
  }
}
