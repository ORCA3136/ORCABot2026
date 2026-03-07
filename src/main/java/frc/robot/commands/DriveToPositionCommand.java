package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.text.FieldPosition;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;

import com.google.flatbuffers.Constants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldPositions;
import frc.robot.Constants.PathPlannerConstants;
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
      Command driveCmd;
      Pose2d target = FieldPositions.kCustomScoringPose;
      DataLogManager.log("DriveToScore-Custom: target=" + target);
      

        // Check if target position needs to be flipped
        var alliance = DriverStation.getAlliance();
        boolean flip = alliance.get() == DriverStation.Alliance.Red;
        if (flip) {
          target = new Pose2d(new Translation2d(16.54 - target.getX(), 8.07 - target.getY()), target.getRotation());
        }

        // Get path velocity heading
        Rotation2d pathVelocityHeading;
        ChassisSpeeds cs = swerve.getFieldVelocity();
        if (swerve.getVelocityMagnitude().in(MetersPerSecond) < 0.25) {
            var diff = target.minus(swerve.getPose()).getTranslation();
            pathVelocityHeading = (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();//.rotateBy(Rotation2d.k180deg);
        } else {
          pathVelocityHeading = new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
        }

        // Create the path waypoints (Start, end)
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
          new Pose2d(swerve.getPose().getTranslation(), pathVelocityHeading),
          target
        );
        // When points are too close pathplanner is unreliable
        if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < 0.01) {
            driveCmd = Commands.print("Auto alignment too close to desired position to continue");
        } 
        // Create the pathplanner command to drive the robot
        else {
          PathPlannerPath path = new PathPlannerPath(
            waypoints, 
            PathPlannerConstants.slowConstraints,
            new IdealStartingState(swerve.getVelocityMagnitude(), swerve.getHeading()), 
            new GoalEndState(0.0, target.getRotation())
          );
          path.preventFlipping = true;
          driveCmd = AutoBuilder.followPath(path);
        }

      // Command driveCmd = swerve.driveToPose(target, TELEOP_CONSTRAINTS)
      //     .withName("DriveToScore-Custom-Path");
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
