// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** Runs the conveyor and kicker together at independent speeds. Stops both when the command ends. */
public class RunConveyorAndKickerCommand extends Command {
  private final ConveyorSubsystem m_conveyorSubsystem;
  private final KickerSubsystem m_KickerSubsystem;
  private final double conveyorSpeed;
  private final double kickerSpeed;

  /**
   * @param conveyorSubsystem The conveyor subsystem
   * @param kickerSubsystem The kicker subsystem
   * @param conveyorVelocity RPM-scale speed for conveyor
   * @param kickerVelocity RPM-scale speed for kicker
   */
  public RunConveyorAndKickerCommand(ConveyorSubsystem conveyorSubsystem, KickerSubsystem kickerSubsystem, double conveyorVelocity, double kickerVelocity) {
    m_conveyorSubsystem = conveyorSubsystem;
    m_KickerSubsystem = kickerSubsystem;
    conveyorSpeed = conveyorVelocity;
    kickerSpeed = kickerVelocity;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyorSubsystem, kickerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  // Motors are commanded every cycle so they recover automatically from CAN bus glitches.
  @Override
  public void execute() {
    m_conveyorSubsystem.setConveyorDutyCycle(conveyorSpeed);
    m_KickerSubsystem.setKickerDutyCycle(kickerSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_conveyorSubsystem.setConveyorDutyCycle(0);
    m_KickerSubsystem.setKickerDutyCycle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
