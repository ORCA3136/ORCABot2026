// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.RunRecursiveIntakeRoutine;

/** name me. */
public class RunIntakeRoutine extends Command {
  private IntakeSubsystem intakeSubsystem;
  private double powerSetPoint;
  //private LaserCan lidarObect;
  private VisionSubsystem lidar;
  private LEDSubsystem led;
  private boolean hasCoral;
  private boolean hasSwitched;
  private int recursionDepth = 0;

  /**
   * Creates a new intake routine.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunIntakeRoutine(IntakeSubsystem intakeSubsystem, double power, VisionSubsystem vision, LEDSubsystem ledSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    powerSetPoint = power;
    lidar = vision;
    led = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, vision);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.setIntakePower(powerSetPoint);
    hasCoral = lidar.getCoralInIntake() < 150;
    hasSwitched = !hasCoral;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hasCoral = lidar.getCoralInIntake() < 150;

    if (lidar.getBottomStatus() == 0 && hasCoral == hasSwitched) {
      powerSetPoint /= -2.5;
      recursionDepth ++;
      hasSwitched = !hasCoral;
      intakeSubsystem.setIntakePower(powerSetPoint);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakePower(0);
    recursionDepth = 0;
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (recursionDepth == 3);
  }
}
