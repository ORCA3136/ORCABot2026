package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NetworkTableNames;
import frc.robot.Constants.VisionConstants;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.results.RawFiducial;

/**
 * Dual Limelight 4 vision subsystem using MegaTag2 pose estimation.
 *
 * <p>This subsystem is "command-free" by design: all vision processing happens in
 * {@link #periodic()}. No command should ever {@code addRequirements()} on this
 * subsystem, because vision fusion must never be interrupted.
 *
 * <h2>What this does</h2>
 * Each cycle, this subsystem:
 * <ol>
 *   <li>Feeds the robot's gyro heading to both Limelights (required for MegaTag2)</li>
 *   <li>Gets pose estimates from both cameras</li>
 *   <li>Runs a 6-stage rejection filter to discard bad measurements</li>
 *   <li>Calculates dynamic standard deviations based on tag distance and count</li>
 *   <li>Fuses accepted measurements into the swerve drive's Kalman filter</li>
 * </ol>
 *
 * <h2>IMU mode lifecycle</h2>
 * Managed automatically — no external commands needed:
 * <ul>
 *   <li><b>Disabled:</b> SyncInternalImu — Limelight syncs its internal IMU to our gyro heading</li>
 *   <li><b>Enabled:</b> InternalImuExternalAssist — Limelight uses its own IMU, assisted by our gyro</li>
 *   <li><b>Settling:</b> Brief pause after mode switch to let the Limelight stabilize</li>
 * </ul>
 */
public class VisionSubsystem extends SubsystemBase {

  private final SwerveSubsystem swerveSubsystem;

  private final Limelight limelightFront;
  private final Limelight limelightBack;

  // We construct PoseEstimate objects directly per camera instead of using
  // limelight.createPoseEstimator(MEGATAG2).getPoseEstimate().
  // Reason: YALL's BotPose enum is a Java singleton — the first camera to call
  // createPoseEstimator binds the internal PoseEstimate to its NT table, and the
  // second camera silently reads the first camera's data. Constructing directly
  // gives each camera its own independent PoseEstimate bound to its own NT table.
  private final PoseEstimate frontPoseEstimate;
  private final PoseEstimate backPoseEstimate;

  // --- IMU mode state machine ---
  private boolean wasEnabled = false;
  private int imuSettleCycleCount = 0;
  private boolean imuSettled = false;
  private int postSettleGraceCount = 0;
  private String imuPhase = "SEEDING";

  // --- Per-camera tracking ---
  private Pose2d lastFrontPose = null;
  private Pose2d lastBackPose = null;
  private double lastFrontAcceptTime = 0;
  private double lastBackAcceptTime = 0;

  private int totalTagCount = 0;

  // --- Cached NetworkTable entries ---
  NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
  private final NetworkTable frontTable = ntInstance.getTable(NetworkTableNames.Vision.kFrontTable);
  private final NetworkTable backTable = ntInstance.getTable(NetworkTableNames.Vision.kBackTable);
  private final NetworkTable visionTable = ntInstance.getTable(NetworkTableNames.Vision.kTable);

  // Front camera NT entries
  private final StructPublisher<Pose2d> frontPosePublisher = frontTable
        .getStructTopic(NetworkTableNames.Vision.kVisionPose, Pose2d.struct).publish();
  private final NetworkTableEntry frontTagCountEntry = frontTable.getEntry(NetworkTableNames.Vision.kTagCount);
  private final NetworkTableEntry frontAvgDistEntry = frontTable.getEntry(NetworkTableNames.Vision.kAvgTagDistance);
  private final NetworkTableEntry frontStdDevEntry = frontTable.getEntry(NetworkTableNames.Vision.kXYStdDev);
  private final NetworkTableEntry frontAcceptedEntry = frontTable.getEntry(NetworkTableNames.Vision.kAccepted);
  private final NetworkTableEntry frontRejectEntry = frontTable.getEntry(NetworkTableNames.Vision.kRejectReason);
  private final NetworkTableEntry frontLatencyEntry = frontTable.getEntry(NetworkTableNames.Vision.kLatencyMs);
  private final NetworkTableEntry frontPoseJumpEntry = frontTable.getEntry(NetworkTableNames.Vision.kPoseJumpMeters);
  private final NetworkTableEntry frontHeadingDevEntry = frontTable.getEntry(NetworkTableNames.Vision.kHeadingDeviationDeg);

  // Back camera NT entries
  private final StructPublisher<Pose2d> backPosePublisher = backTable
        .getStructTopic(NetworkTableNames.Vision.kVisionPose, Pose2d.struct).publish();
  private final NetworkTableEntry backTagCountEntry = backTable.getEntry(NetworkTableNames.Vision.kTagCount);
  private final NetworkTableEntry backAvgDistEntry = backTable.getEntry(NetworkTableNames.Vision.kAvgTagDistance);
  private final NetworkTableEntry backStdDevEntry = backTable.getEntry(NetworkTableNames.Vision.kXYStdDev);
  private final NetworkTableEntry backAcceptedEntry = backTable.getEntry(NetworkTableNames.Vision.kAccepted);
  private final NetworkTableEntry backRejectEntry = backTable.getEntry(NetworkTableNames.Vision.kRejectReason);
  private final NetworkTableEntry backLatencyEntry = backTable.getEntry(NetworkTableNames.Vision.kLatencyMs);
  private final NetworkTableEntry backPoseJumpEntry = backTable.getEntry(NetworkTableNames.Vision.kPoseJumpMeters);
  private final NetworkTableEntry backHeadingDevEntry = backTable.getEntry(NetworkTableNames.Vision.kHeadingDeviationDeg);

  // System-level NT entries
  private final NetworkTableEntry totalTagCountEntry = visionTable.getEntry(NetworkTableNames.Vision.kTotalTagCount);
  private final NetworkTableEntry visionHealthyEntry = visionTable.getEntry(NetworkTableNames.Vision.kVisionHealthy);
  private final NetworkTableEntry imuPhaseEntry = visionTable.getEntry(NetworkTableNames.Vision.kImuPhase);

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;

    limelightFront = new Limelight(VisionConstants.kLimelightFrontName);
    limelightBack = new Limelight(VisionConstants.kLimelightBackName);

    // Configure front camera
    // TODO: Measure camera offset on real robot to nearest 5mm
    limelightFront.getSettings()
        .withLimelightLEDMode(LEDMode.ForceOff)
        .withCameraOffset(new Pose3d(
            VisionConstants.kFrontCamForwardM,
            VisionConstants.kFrontCamLeftM,
            VisionConstants.kFrontCamUpM,
            new Rotation3d(0, Math.toRadians(-VisionConstants.kFrontCamPitchDeg), 0)))
        .withImuMode(ImuMode.SyncInternalImu)
        .save();

    // Configure back camera — 180deg yaw because it faces rearward
    // TODO: Measure camera offset on real robot to nearest 5mm
    limelightBack.getSettings()
        .withLimelightLEDMode(LEDMode.ForceOff)
        .withCameraOffset(new Pose3d(
            VisionConstants.kBackCamForwardM,
            VisionConstants.kBackCamLeftM,
            VisionConstants.kBackCamUpM,
            new Rotation3d(0, Math.toRadians(VisionConstants.kBackCamPitchDeg), Math.toRadians(180))))
        .withImuMode(ImuMode.SyncInternalImu)
        .save();

    // Create per-camera PoseEstimate objects (see class-level comment for why)
    frontPoseEstimate = new PoseEstimate(limelightFront, "botpose_orb_wpiblue", true);
    backPoseEstimate = new PoseEstimate(limelightBack, "botpose_orb_wpiblue", true);
  }

  @Override
  public void periodic() {
    // --- IMU mode state machine ---
    boolean isEnabled = DriverStation.isEnabled();

    // TODO add limelight modes to Robot
    if (!isEnabled && wasEnabled) {
      enterSeedMode();
      wasEnabled = false;
    }
    if (isEnabled && !wasEnabled) {
      enterActiveMode();
      wasEnabled = true;
    }

    // --- Feed heading to both cameras ---
    Rotation3d currentRotation = swerveSubsystem.getGyroRotation3d();

    double rollRateDegPerSec = swerveSubsystem.getPigeon2RollRateDegPerSec();
    double pitchRateDegPerSec = swerveSubsystem.getPigeon2PitchRateDegPerSec();
    double yawRateDegPerSec = swerveSubsystem.getPigeon2YawRateDegPerSec();

    Orientation3d orientation = new Orientation3d(
        currentRotation,
        new AngularVelocity3d(
            DegreesPerSecond.of(rollRateDegPerSec),
            DegreesPerSecond.of(pitchRateDegPerSec),
            DegreesPerSecond.of(yawRateDegPerSec)));

    limelightFront.getSettings().withRobotOrientation(orientation);
    limelightBack.getSettings().withRobotOrientation(orientation);

    // --- Settling check ---
    if (!imuSettled) {
      imuSettleCycleCount++;
      if (imuSettleCycleCount >= VisionConstants.kImuSettleCycles) {
        imuSettled = true;
        postSettleGraceCount = 0;
      }
      imuPhase = "SETTLING";
      publishSystemTelemetry();
      return;
    }

    if (postSettleGraceCount < VisionConstants.kPostSettleGraceCycles) {
      postSettleGraceCount++;
    }

    // --- Process cameras ---
    imuPhase = "ACTIVE";
    totalTagCount = 0;
    processCamera(frontPoseEstimate, true);
    processCamera(backPoseEstimate, false);

    publishSystemTelemetry();
  }

  /**
   * Core vision pipeline for a single camera.
   */
  private void processCamera(PoseEstimate poseEstimateObj, boolean isFront) {
    StructPublisher<Pose2d> posePublisher = isFront ? frontPosePublisher : backPosePublisher;
    NetworkTableEntry tagCountEntry = isFront ? frontTagCountEntry : backTagCountEntry;
    NetworkTableEntry avgDistEntry = isFront ? frontAvgDistEntry : backAvgDistEntry;
    NetworkTableEntry stdDevEntry = isFront ? frontStdDevEntry : backStdDevEntry;
    NetworkTableEntry acceptedEntry = isFront ? frontAcceptedEntry : backAcceptedEntry;
    NetworkTableEntry rejectEntry = isFront ? frontRejectEntry : backRejectEntry;
    NetworkTableEntry latencyEntry = isFront ? frontLatencyEntry : backLatencyEntry;
    NetworkTableEntry poseJumpEntry = isFront ? frontPoseJumpEntry : backPoseJumpEntry;
    NetworkTableEntry headingDevEntry = isFront ? frontHeadingDevEntry : backHeadingDevEntry;

    Optional<PoseEstimate> estimateOpt = poseEstimateObj.getPoseEstimate();

    if (estimateOpt.isEmpty() || !estimateOpt.get().hasData) {
      publishCameraTelemetry(posePublisher, tagCountEntry, avgDistEntry, stdDevEntry,
          acceptedEntry, rejectEntry, latencyEntry, poseJumpEntry, headingDevEntry,
          null, 0, 0, 0, false, "no_data", 0, 0, 0);
      return;
    }

    PoseEstimate estimate = estimateOpt.get();
    Pose2d visionPose = estimate.pose.toPose2d();
    double now = Timer.getFPGATimestamp();

    double headingDeviation = Math.abs(
        visionPose.getRotation().getDegrees() - swerveSubsystem.getHeading().getDegrees());

    // --- Rejection pipeline ---
    String rejectReason = getRejectReason(estimate, visionPose, now, isFront);
    double poseJump = getLastPoseJump(visionPose, isFront);

    if (rejectReason.isEmpty()) {
      // Calculate std devs: scales quadratically with distance, inversely with tag count
      double distanceFactor = estimate.avgTagDist * estimate.avgTagDist;
      double tagFactor = 1.0 / Math.max(1, estimate.tagCount);
      double singleTagPenalty = (estimate.tagCount == 1) ? VisionConstants.kSingleTagPenalty : 1.0;
      double xyStdDev = Math.max(VisionConstants.kMinXYStdDev,
          VisionConstants.kXYStdDevBase * distanceFactor * tagFactor * singleTagPenalty);

      // Effectively infinite — ignore vision heading, trust gyro entirely
      double rotStdDev = 9999.0;

      swerveSubsystem.addVisionMeasurement(
          visionPose,
          estimate.timestampSeconds,
          VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev));

      if (isFront) {
        lastFrontPose = visionPose;
        lastFrontAcceptTime = now;
      } else {
        lastBackPose = visionPose;
        lastBackAcceptTime = now;
      }

      totalTagCount += estimate.tagCount;

      publishCameraTelemetry(posePublisher, tagCountEntry, avgDistEntry, stdDevEntry,
          acceptedEntry, rejectEntry, latencyEntry, poseJumpEntry, headingDevEntry,
          visionPose, estimate.tagCount, estimate.avgTagDist, xyStdDev,
          true, "", estimate.latency, poseJump, headingDeviation);
    } else {
      publishCameraTelemetry(posePublisher, tagCountEntry, avgDistEntry, stdDevEntry,
          acceptedEntry, rejectEntry, latencyEntry, poseJumpEntry, headingDevEntry,
          visionPose, estimate.tagCount, estimate.avgTagDist, 0,
          false, rejectReason, estimate.latency, poseJump, headingDeviation);
    }
  }

  /**
   * Runs the 6-stage rejection filter. Returns empty string if accepted.
   */
  private String getRejectReason(PoseEstimate estimate, Pose2d visionPose, double now, boolean isFront) {
    // 1. Stale timestamp
    double age = now - estimate.timestampSeconds;
    if (age > VisionConstants.kMaxTimestampAgeSec || age < 0) {
      return "stale_timestamp";
    }

    // 2. Field boundary
    double x = visionPose.getX();
    double y = visionPose.getY();
    if (x < VisionConstants.kFieldMinX || x > VisionConstants.kFieldMaxX
        || y < VisionConstants.kFieldMinY || y > VisionConstants.kFieldMaxY) {
      return "out_of_field";
    }

    // 3. High yaw rate
    double yawRate = Math.abs(swerveSubsystem.getPigeon2YawRateDegPerSec());
    if (yawRate > VisionConstants.kMaxYawRateDegPerSec) {
      return "high_yaw_rate";
    }

    // 4. Single tag ambiguity
    if (estimate.tagCount == 1 && estimate.getMaxTagAmbiguity() > VisionConstants.kMaxAmbiguitySingleTag) {
      return "high_ambiguity";
    }

    // 5. Per-camera pose jump
    Pose2d lastPose = isFront ? lastFrontPose : lastBackPose;
    if (lastPose != null) {
      double jump = visionPose.getTranslation().getDistance(lastPose.getTranslation());
      double jumpLimit;
      if (postSettleGraceCount < VisionConstants.kPostSettleGraceCycles) {
        jumpLimit = VisionConstants.kMaxPoseJumpSettling;
      } else if (estimate.tagCount > 1) {
        jumpLimit = VisionConstants.kMaxPoseJumpMultiTag;
      } else {
        jumpLimit = VisionConstants.kMaxPoseJumpSingleTag;
      }
      if (jump > jumpLimit) {
        return "pose_jump";
      }
    }

    // 6. Odometry cross-check
    double odometryJump = visionPose.getTranslation()
        .getDistance(swerveSubsystem.getPose().getTranslation());
    double odometryLimit = (postSettleGraceCount < VisionConstants.kPostSettleGraceCycles)
        ? VisionConstants.kMaxOdometryJumpSettlingM
        : VisionConstants.kMaxOdometryJumpM;
    if (odometryJump > odometryLimit) {
      return "odometry_jump";
    }

    return "";
  }

  private double getLastPoseJump(Pose2d visionPose, boolean isFront) {
    Pose2d lastPose = isFront ? lastFrontPose : lastBackPose;
    if (lastPose == null) return 0;
    return visionPose.getTranslation().getDistance(lastPose.getTranslation());
  }

  // --- IMU mode transitions ---

  private void enterSeedMode() {
    limelightFront.getSettings().withImuMode(ImuMode.SyncInternalImu).save();
    limelightBack.getSettings().withImuMode(ImuMode.SyncInternalImu).save();
    imuSettleCycleCount = 0;
    imuSettled = false;
    postSettleGraceCount = 0;
    imuPhase = "SEEDING";
    lastFrontPose = null;
    lastBackPose = null;
    DataLogManager.log("Vision: entering seed mode (SyncInternalImu)");
  }

  private void enterActiveMode() {
    limelightFront.getSettings().withImuMode(ImuMode.InternalImuExternalAssist).save();
    limelightBack.getSettings().withImuMode(ImuMode.InternalImuExternalAssist).save();
    imuSettleCycleCount = 0;
    imuSettled = false;
    postSettleGraceCount = 0;
    imuPhase = "SETTLING";
    lastFrontPose = null;
    lastBackPose = null;
    DataLogManager.log("Vision: entering active mode, settling for "
        + VisionConstants.kImuSettleCycles + " cycles");
  }

  // --- Utility methods ---

  /** @return true if at least one camera accepted a measurement recently. */
  public boolean isVisionHealthy() {
    double now = Timer.getFPGATimestamp();
    return (now - lastFrontAcceptTime < VisionConstants.kVisionHealthyTimeoutSec)
        || (now - lastBackAcceptTime < VisionConstants.kVisionHealthyTimeoutSec);
  }

  /**
   * Gets the distance to a specific AprilTag from either camera.
   * Useful for variable-distance shooting interpolation.
   *
   * @param tagId The AprilTag ID to find.
   * @return Distance in meters, or -1 if tag not visible or data is stale.
   */
  public double getDistanceToTag(int tagId) {
    double dist = getDistanceToTagFromEstimate(frontPoseEstimate, tagId);
    if (dist >= 0) return dist;
    return getDistanceToTagFromEstimate(backPoseEstimate, tagId);
  }

  private double getDistanceToTagFromEstimate(PoseEstimate poseEstimateWrapper, int tagId) {
    Optional<PoseEstimate> opt = poseEstimateWrapper.getPoseEstimate();
    if (opt.isEmpty() || !opt.get().hasData) return -1;

    double age = Timer.getFPGATimestamp() - opt.get().timestampSeconds;
    if (age > VisionConstants.kTagDistanceStaleSec) return -1;

    for (RawFiducial fiducial : opt.get().rawFiducials) {
      if (fiducial.id == tagId) {
        return fiducial.distToRobot;
      }
    }
    return -1;
  }

  // --- Telemetry ---

  private void publishCameraTelemetry(
      StructPublisher<Pose2d> posePub, NetworkTableEntry tagCountNt, NetworkTableEntry avgDistNt,
      NetworkTableEntry stdDevNt, NetworkTableEntry acceptedNt, NetworkTableEntry rejectNt,
      NetworkTableEntry latencyNt, NetworkTableEntry poseJumpNt, NetworkTableEntry headingDevNt,
      Pose2d pose, int tagCount, double avgTagDist, double xyStdDev,
      boolean accepted, String rejectReason, double latencyMs, double poseJump,
      double headingDeviationDeg) {
    if (pose != null) {
      posePub.set(pose);
    }
    tagCountNt.setDouble(tagCount);
    avgDistNt.setDouble(avgTagDist);
    stdDevNt.setDouble(xyStdDev);
    acceptedNt.setBoolean(accepted);
    rejectNt.setString(rejectReason);
    latencyNt.setDouble(latencyMs);
    poseJumpNt.setDouble(poseJump);
    headingDevNt.setDouble(headingDeviationDeg);
  }

  private void publishSystemTelemetry() {
    totalTagCountEntry.setDouble(totalTagCount);
    visionHealthyEntry.setBoolean(isVisionHealthy());
    imuPhaseEntry.setString(imuPhase);
  }

  @Override
  public void simulationPeriodic() {}

  public Command LLSeedCommand() {
    return Commands.runOnce(() -> enterSeedMode(), (Subsystem[]) null);
  }

  public Command LLActiveCommand() {
    return Commands.runOnce(() -> enterActiveMode(), (Subsystem[]) null);
  }
}
