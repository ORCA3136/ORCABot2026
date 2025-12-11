// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DoubleLidarRoutine extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private IntakeSubsystem intakeSubsystem;
  private ClimberSubsystem climberSubsystem;
  private double powerSetPoint;
  //private LaserCan lidarObect;
  private VisionSubsystem vision;
  private LEDSubsystem led;

  private boolean pulse = false;
  private double time;
  private double commandTime;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DoubleLidarRoutine(IntakeSubsystem intakeSubsystem, double power, VisionSubsystem vision, LEDSubsystem ledSubsystem, ClimberSubsystem climberSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.climberSubsystem = climberSubsystem;
    powerSetPoint = power;
    this.vision = vision;
    led = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.setIntakePower(powerSetPoint);

    time = Timer.getTimestamp();
    commandTime = Timer.getTimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!vision.hasCoralInFunnel()) {
      intakeSubsystem.setIntakePower(-0.1);
      climberSubsystem.setFunnelPower(0);
      time = Timer.getTimestamp();
      commandTime = Timer.getTimestamp();
    } else {
      intakeSubsystem.setIntakePower(powerSetPoint);
    }

    if (!pulse && Timer.getTimestamp() > time + 0.15) {
      climberSubsystem.setFunnelPower(-0.30);
      time = Timer.getTimestamp();
      pulse = !pulse;
    } 
    else if (pulse && Timer.getTimestamp() > time + 0.01) {
      climberSubsystem.setFunnelPower(0);
      time = Timer.getTimestamp();
      pulse = !pulse;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakePower(0);
    climberSubsystem.setFunnelPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return vision.hasCoralInIntake() || Timer.getTimestamp() > commandTime + 0.35;
  }
}
