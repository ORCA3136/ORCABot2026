package frc.robot.commands;

import java.util.Set;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldPositions;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Factory class for drive-to-position commands using PathPlanner pathfinding.
 *
 * <p>Each method returns a deferred command that generates the path at button-press time
 * (not at robot boot), so the robot always pathfinds from its current position.
 *
 * <p>All target poses are defined in blue-origin coordinates. When the alliance is Red,
 * the pose is automatically mirrored by PathPlanner's AutoBuilder (configured in SwerveSubsystem).
 */
public final class DriveToPositionCommand {

  /** Conservative teleop constraints: 2.5 m/s, 2.5 m/s^2 (robot max is 3.05 m/s) */
  private static final PathConstraints TELEOP_CONSTRAINTS =
      new PathConstraints(2.5, 2.5, Math.toRadians(540), Math.toRadians(720));

  private DriveToPositionCommand() {}

  /** Drive to the hub shooting position. */
  public static Command driveToHub(SwerveSubsystem swerve) {
    return deferredDriveTo(swerve, FieldPositions.kHubPose, "Drive To Hub");
  }

  /** Drive to the tower climbing approach position. */
  public static Command driveToTower(SwerveSubsystem swerve) {
    return deferredDriveTo(swerve, FieldPositions.kTowerPose, "Drive To Tower");
  }

  /** Drive to the outpost human player pickup. */
  public static Command driveToOutpost(SwerveSubsystem swerve) {
    return deferredDriveTo(swerve, FieldPositions.kOutpostPose, "Drive To Outpost");
  }

  /** Drive to the depot floor fuel pickup. */
  public static Command driveToDepot(SwerveSubsystem swerve) {
    return deferredDriveTo(swerve, FieldPositions.kDepotPose, "Drive To Depot");
  }

  /** Drive to the left trench entrance. */
  public static Command driveToLeftTrench(SwerveSubsystem swerve) {
    return deferredDriveTo(swerve, FieldPositions.kLeftTrenchPose, "Drive To Left Trench");
  }

  /** Drive to the right trench entrance. */
  public static Command driveToRightTrench(SwerveSubsystem swerve) {
    return deferredDriveTo(swerve, FieldPositions.kRightTrenchPose, "Drive To Right Trench");
  }

  /**
   * Creates a deferred pathfinding command to the given blue-origin pose.
   * The path is generated at runtime so the robot always plans from its current position.
   */
  private static Command deferredDriveTo(SwerveSubsystem swerve, Pose2d bluePose, String name) {
    return Commands.defer(
        () -> swerve.driveToPose(bluePose, TELEOP_CONSTRAINTS),
        Set.of(swerve)).withName(name);
  }
}
