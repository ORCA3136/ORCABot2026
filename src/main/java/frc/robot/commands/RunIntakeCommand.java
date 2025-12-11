// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RunIntakeCommand extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final double powerSetPoint;
  private final LEDSubsystem led;
  //private LaserCan lidarObect;
  private VisionSubsystem lidar;
  private boolean hasCoral;
  private boolean hasSwitched;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunIntakeCommand(IntakeSubsystem intakeSubsystem, double power, VisionSubsystem vision, LEDSubsystem ledSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    powerSetPoint = power;
    lidar = vision;
    led = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, vision,led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.setIntakePower(powerSetPoint);
    hasCoral = lidar.getCoralInIntake() < 150; //What is this doing?
    hasSwitched = !hasCoral;
    led.ChangeLedColor(6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hasCoral = lidar.getCoralInIntake() < 150;
    // if (hasCoral == hasSwitched) {
    //   // Stop the input only when the lidar sensor switches from false to true
    //   hasSwitched = true;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakePower(0);
    // isFinished();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (hasCoral == hasSwitched) {
    //   // Stop the input only when the lidar sensor switches
    //   return true;
    // }
    return false;
  }
}
