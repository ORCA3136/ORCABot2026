// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.LimelightHelpers;
import frc.robot.Constants.Reef;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.function.ToLongBiFunction;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** A command that moves the robot to a side of the reef. */
public class CenterLimelightOnReef extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem m_subsystem;
  private Reef side;
  private String limelight;
  private double tolerance = 1;
  // private double y = 0;
  // private double x = 0; // probably not needed
  // private double theta = 0;

  /**
   * Creates a new CenterLimelightCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CenterLimelightOnReef(SwerveSubsystem subsystem, Reef side) {
    m_subsystem = subsystem;
    this.side = side;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    switch (side) {
      case right:
        limelight = "limelight-left";
        break;
      case left:
        limelight = "limelight-right";
        break;
      case forward:
        break;
    
      default:
        end(false);
        break; // ig
    }
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV(limelight)) {
      double[] values = LimelightHelpers.getCameraPose_TargetSpace(limelight);
      m_subsystem.drive(new Translation2d(values[1]-0.15, values[0]), values[4]/30, false);
    } else {
      end(false);
    }


    
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Math.abs(LimelightHelpers.getTX(limelight)) < tolerance;
  }
}
