package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
// import java.math.*;

import frc.robot.LimelightHelpers;

public class AprilTagFollowCommand extends Command {
  /** Creates a new AprilTagFollowCommand. */

  SwerveSubsystem m_robotDrive;
  String limelightName = "limelight-two";

  public AprilTagFollowCommand(SwerveSubsystem robotDrive) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_robotDrive = robotDrive;

    addRequirements(robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Positive: Target is right of cursor
    double m_tx = LimelightHelpers.getTX(limelightName);
    //Positive: Target is above cursor
    // double m_ty = LimelightHelpers.getTY(limelightName);
    //Bigger number means closer
    double m_ta = LimelightHelpers.getTA(limelightName);

    boolean m_tv = LimelightHelpers.getTV(limelightName);

    double xSpeed = 0.0;
    if (m_tv) {
      if (m_ta < 1.3) {
        xSpeed = -0.2 * (2.7 - m_ta);
      } else if (m_ta > 1.5) {
        xSpeed = 0.2 * m_ta;
      }
    }
    
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("xSpeed").setNumber(xSpeed);

    double rot = 0;
    if (m_tx < -2) {
      rot = -0.5 + 0.04 * m_tx;
    } else if (m_tx > 2) {
      rot = 0.5 + 0.04 * m_tx;
    }
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("rot").setNumber(rot);


    m_robotDrive.drive(new Translation2d(xSpeed, 0), rot, false);

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