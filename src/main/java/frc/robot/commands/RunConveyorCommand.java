// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ConveyorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** Runs the conveyor belt at a fixed speed. Stops when the command ends. */
public class RunConveyorCommand extends Command {
  private final ConveyorSubsystem m_conveyorSubsystem;
  private final double velocity;

  /**
   * @param conveyorSubsystem The conveyor subsystem
   * @param conveyorVelocity RPM-scale speed (positive = toward shooter, negative = toward intake)
   */
  public RunConveyorCommand(ConveyorSubsystem conveyorSubsystem, double conveyorVelocity) {
    m_conveyorSubsystem = conveyorSubsystem;
    velocity = conveyorVelocity;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  // Motor is commanded every cycle so it recovers automatically from CAN bus glitches.
  @Override
  public void execute() {
    m_conveyorSubsystem.setConveyorDutyCycle(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_conveyorSubsystem.setConveyorDutyCycle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
