// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** An example command that uses an example subsystem. */
public class GoToScoreAlgaeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem m_subsystem;
  private boolean isRed = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GoToScoreAlgaeCommand(SwerveSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_subsystem.getPose().getX() > 8.775) {
      isRed = true;
    } else {
      isRed = false;
    }
  }


  

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isRed) {
      m_subsystem.driveToPose(new Pose2d(new Translation2d(9.775, m_subsystem.getPose().getY()), new Rotation2d(180)), null);
    } else {
      m_subsystem.driveToPose(new Pose2d(new Translation2d(7.775, m_subsystem.getPose().getY()), new Rotation2d(0)), null);
    }
  }





  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
