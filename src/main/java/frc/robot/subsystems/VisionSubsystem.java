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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
 *
 * <h2>Why MegaTag2 ignores rotation</h2>
 * MT2 already uses the gyro to resolve heading, so its rotation estimate has no
 * independent information. We set rotation std dev to effectively infinity so the
 * Kalman filter trusts the gyro for heading and only uses vision for XY position.
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
  // Tracks the last accepted vision pose and timestamp for each camera independently.
  // Used for pose jump detection and vision health monitoring.
  private Pose2d lastFrontPose = null;
  private Pose2d lastBackPose = null;
  private double lastFrontAcceptTime = 0;
  private double lastBackAcceptTime = 0;

  // Accumulated tag count for system-level telemetry
  private int totalTagCount = 0;

  // --- Cached NetworkTable entries ---
  // Cached as fields to avoid hash map lookups on every cycle (19 lookups/cycle at 50Hz).
  private final NetworkTable frontTable;
  private final NetworkTable backTable;
  private final NetworkTable visionTable;

  // Front camera NT entries
  private final NetworkTableEntry frontPoseEntry;
  private final NetworkTableEntry frontTagCountEntry;
  private final NetworkTableEntry frontAvgDistEntry;
  private final NetworkTableEntry frontStdDevEntry;
  private final NetworkTableEntry frontAcceptedEntry;
  private final NetworkTableEntry frontRejectEntry;
  private final NetworkTableEntry frontLatencyEntry;
  private final NetworkTableEntry frontPoseJumpEntry;

  // Back camera NT entries
  private final NetworkTableEntry backPoseEntry;
  private final NetworkTableEntry backTagCountEntry;
  private final NetworkTableEntry backAvgDistEntry;
  private final NetworkTableEntry backStdDevEntry;
  private final NetworkTableEntry backAcceptedEntry;
  private final NetworkTableEntry backRejectEntry;
  private final NetworkTableEntry backLatencyEntry;
  private final NetworkTableEntry backPoseJumpEntry;

  // System-level NT entries
  private final NetworkTableEntry totalTagCountEntry;
  private final NetworkTableEntry visionHealthyEntry;
  private final NetworkTableEntry imuPhaseEntry;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;

    limelightFront = new Limelight(VisionConstants.kLimelightFrontName);
    limelightBack = new Limelight(VisionConstants.kLimelightBackName);

    // Configure front camera — LEDs off (AprilTag pipeline doesn't need them)
    limelightFront.getSettings()
        .withLimelightLEDMode(LEDMode.ForceOff)
        .withCameraOffset(new Pose3d(
            VisionConstants.kFrontCamForwardM,
            VisionConstants.kFrontCamLeftM,
            VisionConstants.kFrontCamUpM,
            new Rotation3d(0, Math.toRadians(-VisionConstants.kFrontCamPitchDeg), 0)))
        .withImuMode(ImuMode.SyncInternalImu)
        .save();

    // Configure back camera — 180deg yaw because it faces rearward.
    // TODO: Verify pitch sign empirically. If the back camera can't see tags on
    // the far wall at 3+ meters, try changing -kBackCamPitchDeg to +kBackCamPitchDeg.
    limelightBack.getSettings()
        .withLimelightLEDMode(LEDMode.ForceOff)
        .withCameraOffset(new Pose3d(
            VisionConstants.kBackCamForwardM,
            VisionConstants.kBackCamLeftM,
            VisionConstants.kBackCamUpM,
            new Rotation3d(0, Math.toRadians(-VisionConstants.kBackCamPitchDeg), Math.toRadians(180))))
        .withImuMode(ImuMode.SyncInternalImu)
        .save();

    // Create per-camera PoseEstimate objects (see class-level comment for why)
    frontPoseEstimate = new PoseEstimate(limelightFront, "botpose_orb_wpiblue", true);
    backPoseEstimate = new PoseEstimate(limelightBack, "botpose_orb_wpiblue", true);

    // Cache all NetworkTable entries to avoid string lookups every cycle
    NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    frontTable = ntInstance.getTable(NetworkTableNames.Vision.kFrontTable);
    backTable = ntInstance.getTable(NetworkTableNames.Vision.kBackTable);
    visionTable = ntInstance.getTable(NetworkTableNames.Vision.kTable);

    frontPoseEntry = frontTable.getEntry(NetworkTableNames.Vision.kVisionPose);
    frontTagCountEntry = frontTable.getEntry(NetworkTableNames.Vision.kTagCount);
    frontAvgDistEntry = frontTable.getEntry(NetworkTableNames.Vision.kAvgTagDistance);
    frontStdDevEntry = frontTable.getEntry(NetworkTableNames.Vision.kXYStdDev);
    frontAcceptedEntry = frontTable.getEntry(NetworkTableNames.Vision.kAccepted);
    frontRejectEntry = frontTable.getEntry(NetworkTableNames.Vision.kRejectReason);
    frontLatencyEntry = frontTable.getEntry(NetworkTableNames.Vision.kLatencyMs);
    frontPoseJumpEntry = frontTable.getEntry(NetworkTableNames.Vision.kPoseJumpMeters);

    backPoseEntry = backTable.getEntry(NetworkTableNames.Vision.kVisionPose);
    backTagCountEntry = backTable.getEntry(NetworkTableNames.Vision.kTagCount);
    backAvgDistEntry = backTable.getEntry(NetworkTableNames.Vision.kAvgTagDistance);
    backStdDevEntry = backTable.getEntry(NetworkTableNames.Vision.kXYStdDev);
    backAcceptedEntry = backTable.getEntry(NetworkTableNames.Vision.kAccepted);
    backRejectEntry = backTable.getEntry(NetworkTableNames.Vision.kRejectReason);
    backLatencyEntry = backTable.getEntry(NetworkTableNames.Vision.kLatencyMs);
    backPoseJumpEntry = backTable.getEntry(NetworkTableNames.Vision.kPoseJumpMeters);

    totalTagCountEntry = visionTable.getEntry(NetworkTableNames.Vision.kTotalTagCount);
    visionHealthyEntry = visionTable.getEntry(NetworkTableNames.Vision.kVisionHealthy);
    imuPhaseEntry = visionTable.getEntry(NetworkTableNames.Vision.kImuPhase);
  }

  @Override
  public void periodic() {
    // --- IMU mode state machine ---
    // Automatically transitions between seed and active modes based on robot state.
    // This is self-healing: even if the robot restarts mid-match, the state machine
    // immediately evaluates which mode to be in.
    boolean isEnabled = DriverStation.isEnabled();

    if (!isEnabled && wasEnabled) {
      enterSeedMode();
      wasEnabled = false;
    }

    if (isEnabled && !wasEnabled) {
      enterActiveMode();
      wasEnabled = true;
    }

    // --- Feed heading to both cameras ---
    // MegaTag2 requires the robot's current orientation to resolve AprilTag pose ambiguity.
    // We use the full 3D rotation (pitch, roll, yaw) from the Pigeon2 gyro so the
    // Limelight has accurate data when the robot crosses the Bump (6.5" tall obstacle).
    Rotation3d currentRotation = swerveSubsystem.getGyroRotation3d();

    // Use Pigeon2 yaw rate directly — more accurate than wheel-derived angular velocity
    // because MEMS gyros aren't affected by wheel slip during aggressive turning.
    double yawRateDegPerSec = swerveSubsystem.getPigeon2YawRateDegPerSec();

    Orientation3d orientation = new Orientation3d(
        currentRotation,
        new AngularVelocity3d(
            DegreesPerSecond.of(0),
            DegreesPerSecond.of(0),
            DegreesPerSecond.of(yawRateDegPerSec)));

    // withRobotOrientation() internally calls save()/flush(), so do NOT chain .save()
    // after it — that would cause redundant NT flushes (4 per cycle = 200/sec).
    limelightFront.getSettings().withRobotOrientation(orientation);
    limelightBack.getSettings().withRobotOrientation(orientation);

    // --- Settling check ---
    // After an IMU mode switch, wait for the Limelight's internal IMU to stabilize
    // before trusting any vision measurements.
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

    // Track grace period after settling (briefly relaxed pose jump limits)
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
   *
   * <p>Pipeline stages:
   * <ol>
   *   <li>Get MegaTag2 pose estimate from the camera's NT table</li>
   *   <li>Run rejection filters (any failure = measurement discarded)</li>
   *   <li>Calculate standard deviations based on tag distance and count</li>
   *   <li>Fuse into the swerve drive's Kalman filter pose estimator</li>
   * </ol>
   *
   * @param poseEstimateObj Per-camera PoseEstimate wrapper (reads from that camera's NT)
   * @param isFront         true for front camera, false for back (selects per-camera state)
   */
  private void processCamera(PoseEstimate poseEstimateObj, boolean isFront) {
    NetworkTableEntry poseEntry = isFront ? frontPoseEntry : backPoseEntry;
    NetworkTableEntry tagCountEntry = isFront ? frontTagCountEntry : backTagCountEntry;
    NetworkTableEntry avgDistEntry = isFront ? frontAvgDistEntry : backAvgDistEntry;
    NetworkTableEntry stdDevEntry = isFront ? frontStdDevEntry : backStdDevEntry;
    NetworkTableEntry acceptedEntry = isFront ? frontAcceptedEntry : backAcceptedEntry;
    NetworkTableEntry rejectEntry = isFront ? frontRejectEntry : backRejectEntry;
    NetworkTableEntry latencyEntry = isFront ? frontLatencyEntry : backLatencyEntry;
    NetworkTableEntry poseJumpEntry = isFront ? frontPoseJumpEntry : backPoseJumpEntry;

    Optional<PoseEstimate> estimateOpt = poseEstimateObj.getPoseEstimate();

    if (estimateOpt.isEmpty() || !estimateOpt.get().hasData) {
      publishCameraTelemetry(poseEntry, tagCountEntry, avgDistEntry, stdDevEntry,
          acceptedEntry, rejectEntry, latencyEntry, poseJumpEntry,
          null, 0, 0, 0, false, "no_data", 0, 0);
      return;
    }

    PoseEstimate estimate = estimateOpt.get();
    Pose2d visionPose = estimate.pose.toPose2d();
    double now = Timer.getFPGATimestamp();

    // --- Rejection pipeline ---
    // Each filter short-circuits: first failure sets the reason and skips the rest.
    // Reject reasons are published to NT for real-time debugging in AdvantageScope.
    String rejectReason = getRejectReason(estimate, visionPose, now, isFront);
    double poseJump = getLastPoseJump(visionPose, isFront);

    // --- If accepted, calculate std devs and fuse ---
    if (rejectReason.isEmpty()) {
      // Standard deviation formula: controls Kalman filter trust in this measurement.
      // Higher std dev = less trust. Scales quadratically with distance (farther tags
      // are exponentially less accurate) and inversely with tag count (multi-tag solves
      // are more reliable). Single-tag solves get an extra penalty because they have
      // an ambiguity failure mode where two mirror-image poses are both valid.
      double distanceFactor = estimate.avgTagDist * estimate.avgTagDist;
      double tagFactor = 1.0 / Math.max(1, estimate.tagCount);
      double singleTagPenalty = (estimate.tagCount == 1) ? VisionConstants.kSingleTagPenalty : 1.0;
      double xyStdDev = Math.max(VisionConstants.kMinXYStdDev,
          VisionConstants.kXYStdDevBase * distanceFactor * tagFactor * singleTagPenalty);

      // Effectively infinite — tells the Kalman filter to ignore vision heading entirely.
      // Using 9999.0 instead of Double.MAX_VALUE to avoid Infinity when the estimator
      // internally squares the std devs for the covariance matrix.
      double rotStdDev = 9999.0;

      swerveSubsystem.addVisionMeasurement(
          visionPose,
          estimate.timestampSeconds,
          VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev));

      // Update per-camera tracking state
      if (isFront) {
        lastFrontPose = visionPose;
        lastFrontAcceptTime = now;
      } else {
        lastBackPose = visionPose;
        lastBackAcceptTime = now;
      }

      totalTagCount += estimate.tagCount;

      publishCameraTelemetry(poseEntry, tagCountEntry, avgDistEntry, stdDevEntry,
          acceptedEntry, rejectEntry, latencyEntry, poseJumpEntry,
          visionPose, estimate.tagCount, estimate.avgTagDist, xyStdDev,
          true, "", estimate.latency, poseJump);
    } else {
      publishCameraTelemetry(poseEntry, tagCountEntry, avgDistEntry, stdDevEntry,
          acceptedEntry, rejectEntry, latencyEntry, poseJumpEntry,
          visionPose, estimate.tagCount, estimate.avgTagDist, 0,
          false, rejectReason, estimate.latency, poseJump);
    }
  }

  /**
   * Runs the 6-stage rejection filter and returns the reason string (empty if accepted).
   * Extracted to a method for readability — each filter returns immediately on failure.
   */
  private String getRejectReason(PoseEstimate estimate, Pose2d visionPose, double now, boolean isFront) {
    // 1. Stale timestamp — measurement is too old to be useful
    double age = now - estimate.timestampSeconds;
    if (age > VisionConstants.kMaxTimestampAgeSec || age < 0) {
      return "stale_timestamp";
    }

    // 2. Field boundary — pose is outside the field, must be a bad solve
    double x = visionPose.getX();
    double y = visionPose.getY();
    if (x < VisionConstants.kFieldMinX || x > VisionConstants.kFieldMaxX
        || y < VisionConstants.kFieldMinY || y > VisionConstants.kFieldMaxY) {
      return "out_of_field";
    }

    // 3. High yaw rate — fast spinning causes heading/image timestamp mismatch
    double yawRate = Math.abs(swerveSubsystem.getPigeon2YawRateDegPerSec());
    if (yawRate > VisionConstants.kMaxYawRateDegPerSec) {
      return "high_yaw_rate";
    }

    // 4. Single tag ambiguity — solver isn't confident which of 2 mirror solutions is correct
    if (estimate.tagCount == 1 && estimate.getMaxTagAmbiguity() > VisionConstants.kMaxAmbiguitySingleTag) {
      return "high_ambiguity";
    }

    // 5. Per-camera pose jump — vision pose moved too far from last accepted pose
    Pose2d lastPose = isFront ? lastFrontPose : lastBackPose;
    if (lastPose != null) {
      double poseJump = visionPose.getTranslation().getDistance(lastPose.getTranslation());
      double jumpLimit;
      if (postSettleGraceCount < VisionConstants.kPostSettleGraceCycles) {
        jumpLimit = VisionConstants.kMaxPoseJumpSettling;
      } else if (estimate.tagCount > 1) {
        jumpLimit = VisionConstants.kMaxPoseJumpMultiTag;
      } else {
        jumpLimit = VisionConstants.kMaxPoseJumpSingleTag;
      }
      if (poseJump > jumpLimit) {
        return "pose_jump";
      }
    }

    // 6. Odometry cross-check — vision pose disagrees with current odometry estimate.
    // Catches bad solves after camera occlusion when lastPose is stale but odometry is current.
    double odometryJump = visionPose.getTranslation()
        .getDistance(swerveSubsystem.getPose().getTranslation());
    if (odometryJump > VisionConstants.kMaxOdometryJumpM) {
      return "odometry_jump";
    }

    return ""; // All filters passed — measurement accepted
  }

  /** Helper to compute the pose jump distance for telemetry, without rejecting. */
  private double getLastPoseJump(Pose2d visionPose, boolean isFront) {
    Pose2d lastPose = isFront ? lastFrontPose : lastBackPose;
    if (lastPose == null) return 0;
    return visionPose.getTranslation().getDistance(lastPose.getTranslation());
  }

  // --- IMU mode transitions ---
  // .save() is needed here because withImuMode() does NOT auto-flush to NT.
  // (This is different from withRobotOrientation() which does auto-flush.)

  /** Enters seed mode: Limelight syncs its internal IMU to our gyro heading. */
  private void enterSeedMode() {
    limelightFront.getSettings().withImuMode(ImuMode.SyncInternalImu).save();
    limelightBack.getSettings().withImuMode(ImuMode.SyncInternalImu).save();
    imuSettleCycleCount = 0;
    imuSettled = false;
    postSettleGraceCount = 0;
    imuPhase = "SEEDING";
    // Clear stale pose tracking — robot may have been repositioned while disabled
    lastFrontPose = null;
    lastBackPose = null;
  }

  /** Enters active mode: Limelight uses its own IMU, assisted by our gyro heading. */
  private void enterActiveMode() {
    limelightFront.getSettings().withImuMode(ImuMode.InternalImuExternalAssist).save();
    limelightBack.getSettings().withImuMode(ImuMode.InternalImuExternalAssist).save();
    imuSettleCycleCount = 0;
    imuSettled = false;
    postSettleGraceCount = 0;
    imuPhase = "SETTLING";
    // Clear stale pose tracking — robot may have moved since last enabled
    lastFrontPose = null;
    lastBackPose = null;
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
   * Checks the most recent frame data from processCamera().
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
    // Re-read the estimate to get fresh data, and verify it's recent (within 2 cycles)
    Optional<PoseEstimate> opt = poseEstimateWrapper.getPoseEstimate();
    if (opt.isEmpty() || !opt.get().hasData) return -1;

    double age = Timer.getFPGATimestamp() - opt.get().timestampSeconds;
    if (age > 0.05) return -1; // Stale data (>2.5 cycles old)

    for (RawFiducial fiducial : opt.get().rawFiducials) {
      if (fiducial.id == tagId) {
        return fiducial.distToRobot;
      }
    }
    return -1;
  }

  // --- Telemetry ---
  // Published to NetworkTables for live monitoring in AdvantageScope.
  // Per-camera: pose, tag count, distance, std dev, accept/reject status.
  // System-level: total tags, health status, IMU phase.

  private void publishCameraTelemetry(
      NetworkTableEntry poseNt, NetworkTableEntry tagCountNt, NetworkTableEntry avgDistNt,
      NetworkTableEntry stdDevNt, NetworkTableEntry acceptedNt, NetworkTableEntry rejectNt,
      NetworkTableEntry latencyNt, NetworkTableEntry poseJumpNt,
      Pose2d pose, int tagCount, double avgTagDist, double xyStdDev,
      boolean accepted, String rejectReason, double latencyMs, double poseJump) {
    if (pose != null) {
      poseNt.setDoubleArray(new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() });
    }
    tagCountNt.setDouble(tagCount);
    avgDistNt.setDouble(avgTagDist);
    stdDevNt.setDouble(xyStdDev);
    acceptedNt.setBoolean(accepted);
    rejectNt.setString(rejectReason);
    latencyNt.setDouble(latencyMs);
    poseJumpNt.setDouble(poseJump);
  }

  private void publishSystemTelemetry() {
    totalTagCountEntry.setDouble(totalTagCount);
    visionHealthyEntry.setBoolean(isVisionHealthy());
    imuPhaseEntry.setString(imuPhase);
  }

  @Override
  public void simulationPeriodic() {}
}
