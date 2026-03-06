package frc.robot.commands;

import java.text.FieldPosition;
import java.util.Set;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldPositions;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;

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

    /** Drive to the right trench entrance. */
  public static Command driveToTestPosition(SwerveSubsystem swerve) {
    return deferredDriveTo(swerve, FieldPositions.kTestPosition, "Drive To Test Position");
  }

  /**
   * Rotate in place to face the hub. Uses a PID controller instead of PathPlanner
   * pathfinding, since pathfinding finishes instantly when there's no translational
   * distance to cover.
   */
  public static Command aimAtHub(SwerveSubsystem swerve) {
    PIDController pid = new PIDController(5.0, 0, 0.3);
    pid.enableContinuousInput(-Math.PI, Math.PI);
    pid.setTolerance(Math.toRadians(2));

    return Commands.defer(() -> {
      Translation2d robotPos = swerve.getPose().getTranslation();
      Translation2d hubPos = swerve.getAlliance() == Alliance.Red
          ? FieldPositions.kRedFieldElements.get(0)
          : FieldPositions.kBlueFieldElements.get(0);
      double targetRad = Math.atan2(
          hubPos.getY() - robotPos.getY(),
          hubPos.getX() - robotPos.getX());

      pid.reset();

      return Commands.run(() -> {
            double omega = pid.calculate(swerve.getHeadingRadians(), targetRad);
            swerve.drive(new Translation2d(0, 0), omega, true);
          }, swerve)
          .until(pid::atSetpoint)
          .withTimeout(3.0)
          .finallyDo(() -> swerve.drive(new Translation2d(0, 0), 0, true));
    }, Set.of(swerve)).withName("Aim At Hub");
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

  // ── Scoring position commands ──────────────────────────────────────

  private static final double FIELD_LENGTH_M = 16.54;

  /** Drive to nearest close scoring position while spinning up shooter. */
  public static Command driveToScoreClose(SwerveSubsystem swerve, ShooterSubsystem shooter,
                                           BooleanSupplier driverOverride) {
    return driveToScoringPosition("Close",
        FieldPositions.kScoreCloseLeft, FieldPositions.kScoreCloseRight,
        swerve, shooter, driverOverride);
  }

  /** Drive to nearest trench scoring position while spinning up shooter. */
  public static Command driveToScoreTrench(SwerveSubsystem swerve, ShooterSubsystem shooter,
                                            BooleanSupplier driverOverride) {
    return driveToScoringPosition("Trench",
        FieldPositions.kScoreTrenchLeft, FieldPositions.kScoreTrenchRight,
        swerve, shooter, driverOverride);
  }

  /** Drive to nearest far scoring position while spinning up shooter. */
  public static Command driveToScoreFar(SwerveSubsystem swerve, ShooterSubsystem shooter,
                                         BooleanSupplier driverOverride) {
    return driveToScoringPosition("Far",
        FieldPositions.kScoreFarLeft, FieldPositions.kScoreFarRight,
        swerve, shooter, driverOverride);
  }

  /** Drive to the custom tested scoring position while spinning up shooter. */
  public static Command driveToScoreCustom(SwerveSubsystem swerve, ShooterSubsystem shooter,
                                            BooleanSupplier driverOverride) {
    return Commands.defer(() -> {
      Pose2d target = FieldPositions.kCustomScoringPose;
      DataLogManager.log("DriveToScore-Custom: target=" + target);
      Command driveCmd = swerve.driveToPose(target, TELEOP_CONSTRAINTS)
          .withName("DriveToScore-Custom-Path");
      Command spinUpCmd = Commands.run(() -> shooter.setShooterMap(), shooter)
          .withName("DriveToScore-Custom-SpinUp");
      return driveCmd.deadlineFor(spinUpCmd);
    }, Set.of(swerve, shooter))
    .until(driverOverride)
    .withTimeout(FieldPositions.kScoringDriveTimeoutSec)
    .finallyDo(interrupted -> {
      DataLogManager.log("DriveToScore-Custom: " + (interrupted ? "CANCELLED" : "ARRIVED"));
    })
    .withName("DriveToScore-Custom");
  }

  /**
   * Shared implementation: drive to nearest left/right variant, spin up shooter in parallel.
   * Cancels on driver stick input or timeout.
   */
  private static Command driveToScoringPosition(
      String name, Pose2d leftBlue, Pose2d rightBlue,
      SwerveSubsystem swerve, ShooterSubsystem shooter,
      BooleanSupplier driverOverride) {

    return Commands.defer(() -> {
      // Pick nearest variant — must compare in robot's actual coordinate frame
      Pose2d target = nearestVariant(swerve, leftBlue, rightBlue);

      DataLogManager.log("DriveToScore-" + name + ": target=" + target);

      // Drive to position (PathPlanner handles alliance flip)
      Command driveCmd = swerve.driveToPose(target, TELEOP_CONSTRAINTS)
          .withName("DriveToScore-" + name + "-Path");

      // Spin up shooter from distance map — dynamically updates as robot moves
      Command spinUpCmd = Commands.run(() -> shooter.setShooterMap(), shooter)
          .withName("DriveToScore-" + name + "-SpinUp");

      // Drive is the deadline — when we arrive, stop the command group
      // (shooter keeps its last RPM target, driver can shoot immediately)
      return driveCmd.deadlineFor(spinUpCmd);

    }, Set.of(swerve, shooter))
    .until(driverOverride)
    .withTimeout(FieldPositions.kScoringDriveTimeoutSec)
    .finallyDo(interrupted -> {
      DataLogManager.log("DriveToScore-" + name + ": "
          + (interrupted ? "CANCELLED" : "ARRIVED"));
    })
    .withName("DriveToScore-" + name);
  }

  /**
   * Returns whichever pose is closer to the robot's current position.
   * Both poses are blue-origin. For Red alliance, we flip them before
   * distance comparison (robot pose is blue-origin but physically on red side).
   * The ORIGINAL blue-origin pose is returned — PathPlanner handles the real flip.
   */
  private static Pose2d nearestVariant(SwerveSubsystem swerve, Pose2d leftBlue, Pose2d rightBlue) {
    Translation2d current = swerve.getPose().getTranslation();

    Translation2d leftCompare = leftBlue.getTranslation();
    Translation2d rightCompare = rightBlue.getTranslation();

    // On Red alliance, actual target positions are mirrored: x' = fieldLength - x, y' = y
    if (swerve.getAlliance() == Alliance.Red) {
      leftCompare = new Translation2d(FIELD_LENGTH_M - leftCompare.getX(), leftCompare.getY());
      rightCompare = new Translation2d(FIELD_LENGTH_M - rightCompare.getX(), rightCompare.getY());
    }

    double distLeft = current.getDistance(leftCompare);
    double distRight = current.getDistance(rightCompare);
    return distLeft <= distRight ? leftBlue : rightBlue;
  }
}
