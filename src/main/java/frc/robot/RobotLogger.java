package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.urcl.URCL;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public final class RobotLogger {

  private static ShooterSubsystem shooterSubsystem;
  private static HoodSubsystem hoodSubsystem;
  private static KickerSubsystem kickerSubsystem;
  private static SwerveSubsystem swerveSubsystem;

  // Shot log entries (written directly to WPILOG, bypasses NT)
  private static DoubleLogEntry shotTimestamp;
  private static DoubleLogEntry shotShooterActualRPM;
  private static DoubleLogEntry shotShooterTargetRPM;
  private static DoubleLogEntry shotShooterRampedSetpoint;
  private static DoubleLogEntry shotHoodAngleDegrees;
  private static DoubleLogEntry shotHoodRotations;
  private static DoubleLogEntry shotKickerVelocity;
  private static DoubleLogEntry shotRobotX;
  private static DoubleLogEntry shotRobotY;
  private static DoubleLogEntry shotRobotHeadingDeg;
  private static DoubleLogEntry shotDistanceToHub;

  private RobotLogger() {}

  /**
   * Initialize data logging.
   * @param useUSB true to write logs to USB stick (/U), false for roboRIO internal storage
   */
  public static void init(boolean useUSB) {
    if (useUSB) {
      DataLogManager.start("/U");
    } else {
      DataLogManager.start();
    }
    DataLogManager.logNetworkTables(true);

    // Log all REV SparkMax/SparkFlex CAN data to WPILOG
    URCL.start();

    // Log match metadata
    DataLogManager.log("Match: " + DriverStation.getEventName()
        + " " + DriverStation.getMatchType()
        + " #" + DriverStation.getMatchNumber());
    DataLogManager.log("Alliance: " + DriverStation.getAlliance().orElse(null));

    // Set up shot log entries under "Shots/" group
    DataLog log = DataLogManager.getLog();
    shotTimestamp = new DoubleLogEntry(log, "Shots/Timestamp");
    shotShooterActualRPM = new DoubleLogEntry(log, "Shots/ShooterActualRPM");
    shotShooterTargetRPM = new DoubleLogEntry(log, "Shots/ShooterTargetRPM");
    shotShooterRampedSetpoint = new DoubleLogEntry(log, "Shots/ShooterRampedSetpoint");
    shotHoodAngleDegrees = new DoubleLogEntry(log, "Shots/HoodAngleDegrees");
    shotHoodRotations = new DoubleLogEntry(log, "Shots/HoodRotations");
    shotKickerVelocity = new DoubleLogEntry(log, "Shots/KickerVelocityRPM");
    shotRobotX = new DoubleLogEntry(log, "Shots/RobotX");
    shotRobotY = new DoubleLogEntry(log, "Shots/RobotY");
    shotRobotHeadingDeg = new DoubleLogEntry(log, "Shots/RobotHeadingDeg");
    shotDistanceToHub = new DoubleLogEntry(log, "Shots/DistanceToHub");
  }

  /** Register subsystem references so logShot() can pull live data. */
  public static void registerSubsystems(ShooterSubsystem shooter, HoodSubsystem hood,
                                         KickerSubsystem kicker, SwerveSubsystem swerve) {
    shooterSubsystem = shooter;
    hoodSubsystem = hood;
    kickerSubsystem = kicker;
    swerveSubsystem = swerve;
  }

  /** Register command start/stop logging with the CommandScheduler. */
  public static void registerCommandLogging() {
    CommandScheduler.getInstance().onCommandInitialize(
        cmd -> DataLogManager.log("CMD+ " + cmd.getName()));
    CommandScheduler.getInstance().onCommandFinish(
        cmd -> DataLogManager.log("CMD- " + cmd.getName()));
  }

  /**
   * Capture a shot snapshot. Writes shooter RPM, hood angle, kicker velocity,
   * robot pose, and distance to hub into the "Shots/" log group.
   * Called by KickerSubsystem when a shot is detected.
   */
  public static void logShot() {
    double now = Timer.getFPGATimestamp();
    shotTimestamp.append(now);

    if (shooterSubsystem != null) {
      shotShooterActualRPM.append(shooterSubsystem.getShooterVelocity());
      shotShooterTargetRPM.append(shooterSubsystem.getShooterTarget());
      shotShooterRampedSetpoint.append(shooterSubsystem.getRampedSetpoint());
    }

    if (hoodSubsystem != null) {
      shotHoodAngleDegrees.append(Math.toDegrees(hoodSubsystem.getHoodAngle()));
      shotHoodRotations.append(hoodSubsystem.getHoodMotorRotations());
    }

    if (kickerSubsystem != null) {
      shotKickerVelocity.append(kickerSubsystem.getKickerVelocity());
    }

    if (swerveSubsystem != null) {
      Pose2d pose = swerveSubsystem.getPose();
      shotRobotX.append(pose.getX());
      shotRobotY.append(pose.getY());
      shotRobotHeadingDeg.append(pose.getRotation().getDegrees());

      Translation2d toHub = swerveSubsystem.getTranslationToFieldElement(
          SwerveSubsystem.FieldTargets.kHub, false);
      shotDistanceToHub.append(toHub != null ? toHub.getNorm() : -1);
    }

    DataLogManager.log("SHOT fired");
  }

  /** Convenience wrapper for DataLogManager.log(). */
  public static void log(String message) {
    DataLogManager.log(message);
  }
}
