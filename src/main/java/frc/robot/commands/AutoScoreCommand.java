// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AutoScoreCommand extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final double powerSetPoint;
  private boolean ejectingCoral = false;
  private boolean hasScored = false;

  private double commandTimer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoScoreCommand(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem, double power, VisionSubsystem vision) {
    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    powerSetPoint = power;
    visionSubsystem = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, elevatorSubsystem, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Start of command");
    commandTimer = Timer.getTimestamp();
    ejectingCoral = false;
    hasScored = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (ejectingCoral && !visionSubsystem.hasCoralInIntake() && Timer.getTimestamp() > commandTimer + 0.25) {
      System.out.println("Command ended naturally");
      hasScored = true;
    }

    if (elevatorSubsystem.atScoringPosition()) {
      ejectingCoral = true;
      intakeSubsystem.setIntakePower(powerSetPoint);
    } else {
      commandTimer = Timer.getTimestamp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakePower(0);
    System.out.println("End of command interrupted: " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (hasScored) System.out.println("Command ended naturally");
    if (Timer.getTimestamp() > commandTimer + 1) System.out.println("Command timed out");

    return hasScored || Timer.getTimestamp() > commandTimer + 1;
  }
}
