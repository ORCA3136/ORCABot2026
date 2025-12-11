// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DriveToPoseCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem swerve;
  private final Pose2d targetpose;
  private final Pose2d[] midpose;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveToPoseCommand(SwerveSubsystem subsystem, Pose2d pose, Pose2d[] midpose) {
    this.swerve = subsystem;
    this.targetpose = pose;
    this.midpose = midpose;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (midpose.length == 0) {
      swerve.driveToPose(targetpose, Constants.PathPlannerConstants.slowConstraints, 0);
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
