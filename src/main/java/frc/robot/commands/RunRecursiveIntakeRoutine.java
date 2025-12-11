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

/** name me. */
public class RunRecursiveIntakeRoutine extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final double powerSetPoint;
  //private LaserCan lidarObect;
  private VisionSubsystem lidar;
  private LEDSubsystem led;
  private boolean hasCoral;
  private boolean hasSwitched;
  private int recursionDepth = 1;

  /**
   * Creates a new intake routine.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunRecursiveIntakeRoutine(IntakeSubsystem intakeSubsystem, double power, VisionSubsystem vision, LEDSubsystem ledSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    powerSetPoint = power;
    lidar = vision;
    led = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, vision);
  }

  public RunRecursiveIntakeRoutine(IntakeSubsystem intakeSubsystem, double power, VisionSubsystem vision,LEDSubsystem ledSubsystem, int r) {
    this(intakeSubsystem, power, vision,ledSubsystem);
    recursionDepth = r;
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
    
      // if (hasCoral == hasSwitched) {
      //   // Stop the input only when the lidar sensor switches from false to true
      //   hasSwitched = true;
      // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakePower(0);
    if (!interrupted && recursionDepth < 3) {
      new RunRecursiveIntakeRoutine(intakeSubsystem, powerSetPoint/-2.5, lidar, led,recursionDepth+1).schedule();
    }
    
    // isFinished();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (hasCoral == hasSwitched || (recursionDepth == 1 && hasCoral)) { //
      // Stop the input only when the lidar sensor switches to true
      led.ChangeLedColor(3);
      return true;
    }
    return false;
  }
}
