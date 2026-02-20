package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
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
 * Feeds gyro heading to both cameras, runs rejection filtering on pose estimates,
 * calculates dynamic standard deviations, and fuses accepted measurements into
 * the swerve drive pose estimator.
 *
 * IMU mode transitions are handled automatically based on DriverStation state:
 * - Disabled: SyncInternalImu (seed mode)
 * - Enabled: InternalImuExternalAssist (active mode)
 */
public class VisionSubsystem extends SubsystemBase {

  private final SwerveSubsystem swerveSubsystem;

  private final Limelight limelightFront;
  private final Limelight limelightBack;

  // Per-camera PoseEstimate objects — avoids YALL BotPose singleton bug
  // where both cameras share one PoseEstimate bound to the first caller.
  private final PoseEstimate frontPoseEstimate;
  private final PoseEstimate backPoseEstimate;

  // IMU mode state machine
  private boolean wasEnabled = false;
  private int imuSettleCycleCount = 0;
  private String imuPhase = "SEEDING";

  // Vision health tracking
  private double lastFrontAcceptTime = 0;
  private double lastBackAcceptTime = 0;

  // Pose jump tracking (previous accepted poses per camera)
  private Pose2d lastFrontPose = null;
  private Pose2d lastBackPose = null;

  // Telemetry — per-camera totals for system-level publishing
  private int totalTagCount = 0;

  // NetworkTables
  private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
  private final NetworkTable frontTable = ntInstance.getTable(NetworkTableNames.Vision.kFrontTable);
  private final NetworkTable backTable = ntInstance.getTable(NetworkTableNames.Vision.kBackTable);
  private final NetworkTable visionTable = ntInstance.getTable(NetworkTableNames.Vision.kTable);

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;

    limelightFront = new Limelight(VisionConstants.kLimelightFrontName);
    limelightBack = new Limelight(VisionConstants.kLimelightBackName);

    // Configure front camera
    limelightFront.getSettings()
        .withLimelightLEDMode(LEDMode.ForceOff)
        .withCameraOffset(new Pose3d(
            VisionConstants.kFrontCamForwardM,
            VisionConstants.kFrontCamLeftM,
            VisionConstants.kFrontCamUpM,
            new Rotation3d(0, Math.toRadians(-VisionConstants.kFrontCamPitchDeg), 0)))
        .withImuMode(ImuMode.SyncInternalImu)
        .save();

    // Configure back camera
    limelightBack.getSettings()
        .withLimelightLEDMode(LEDMode.ForceOff)
        .withCameraOffset(new Pose3d(
            VisionConstants.kBackCamForwardM,
            VisionConstants.kBackCamLeftM,
            VisionConstants.kBackCamUpM,
            new Rotation3d(0, Math.toRadians(-VisionConstants.kBackCamPitchDeg), Math.toRadians(180))))
        .withImuMode(ImuMode.SyncInternalImu)
        .save();

    // Create per-camera PoseEstimate objects directly to avoid the YALL
    // BotPose enum singleton bug (both cameras would share one PoseEstimate).
    frontPoseEstimate = new PoseEstimate(limelightFront, "botpose_orb_wpiblue", true);
    backPoseEstimate = new PoseEstimate(limelightBack, "botpose_orb_wpiblue", true);
  }

  @Override
  public void periodic() {
    // --- IMU mode state machine ---
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
    Rotation3d currentRotation = new Rotation3d(0, 0, swerveSubsystem.getPose().getRotation().getRadians());
    double yawRateDegPerSec = Math.toDegrees(
        swerveSubsystem.getSwerveDrive().getRobotVelocity().omegaRadiansPerSecond);

    Orientation3d orientation = new Orientation3d(
        currentRotation,
        new AngularVelocity3d(
            DegreesPerSecond.of(0),
            DegreesPerSecond.of(0),
            DegreesPerSecond.of(yawRateDegPerSec)));

    limelightFront.getSettings().withRobotOrientation(orientation).save();
    limelightBack.getSettings().withRobotOrientation(orientation).save();

    // --- Settling check ---
    if (imuSettleCycleCount < VisionConstants.kImuSettleCycles) {
      imuSettleCycleCount++;
      imuPhase = "SETTLING";
      publishSystemTelemetry();
      return;
    }

    // --- Process cameras ---
    imuPhase = "ACTIVE";
    totalTagCount = 0;
    imuSettleCycleCount++;
    processCamera(frontPoseEstimate, frontTable, true);
    processCamera(backPoseEstimate, backTable, false);

    publishSystemTelemetry();
  }

  /**
   * Core vision pipeline for a single camera.
   * Gets MT2 estimate, runs rejection filters, calculates std devs, and fuses if accepted.
   */
  private void processCamera(PoseEstimate poseEstimateObj, NetworkTable cameraTable, boolean isFront) {
    String rejectReason = "";

    Optional<PoseEstimate> estimateOpt = poseEstimateObj.getPoseEstimate();

    if (estimateOpt.isEmpty() || !estimateOpt.get().hasData) {
      publishCameraTelemetry(cameraTable, null, 0, 0, 0, false, "no_data", 0, 0);
      return;
    }

    PoseEstimate estimate = estimateOpt.get();
    Pose2d visionPose = estimate.pose.toPose2d();
    double now = Timer.getFPGATimestamp();

    // --- Rejection pipeline (short-circuit) ---

    // 1. Stale timestamp
    double age = now - estimate.timestampSeconds;
    if (age > VisionConstants.kMaxTimestampAgeSec || age < 0) {
      rejectReason = "stale_timestamp";
    }

    // 2. Field boundary check
    if (rejectReason.isEmpty()) {
      double x = visionPose.getX();
      double y = visionPose.getY();
      if (x < VisionConstants.kFieldMinX || x > VisionConstants.kFieldMaxX
          || y < VisionConstants.kFieldMinY || y > VisionConstants.kFieldMaxY) {
        rejectReason = "out_of_field";
      }
    }

    // 3. High yaw rate
    if (rejectReason.isEmpty()) {
      double yawRate = Math.abs(Math.toDegrees(
          swerveSubsystem.getSwerveDrive().getRobotVelocity().omegaRadiansPerSecond));
      if (yawRate > VisionConstants.kMaxYawRateDegPerSec) {
        rejectReason = "high_yaw_rate";
      }
    }

    // 4. Single tag ambiguity
    if (rejectReason.isEmpty() && estimate.tagCount == 1) {
      if (estimate.getMaxTagAmbiguity() > VisionConstants.kMaxAmbiguitySingleTag) {
        rejectReason = "high_ambiguity";
      }
    }

    // 5. Pose jump check
    double poseJump = 0;
    if (rejectReason.isEmpty()) {
      Pose2d lastPose = isFront ? lastFrontPose : lastBackPose;
      if (lastPose != null) {
        poseJump = visionPose.getTranslation().getDistance(lastPose.getTranslation());
        double jumpLimit;
        if (imuSettleCycleCount <= VisionConstants.kImuSettleCycles + 5) {
          jumpLimit = VisionConstants.kMaxPoseJumpSettling;
        } else if (estimate.tagCount > 1) {
          jumpLimit = VisionConstants.kMaxPoseJumpMultiTag;
        } else {
          jumpLimit = VisionConstants.kMaxPoseJumpSingleTag;
        }
        if (poseJump > jumpLimit) {
          rejectReason = "pose_jump";
        }
      }
    }

    // --- If accepted, calculate std devs and fuse ---
    if (rejectReason.isEmpty()) {
      double distanceFactor = estimate.avgTagDist * estimate.avgTagDist;
      double tagFactor = 1.0 / Math.max(1, estimate.tagCount);
      double xyStdDev = Math.max(VisionConstants.kMinXYStdDev,
          VisionConstants.kXYStdDevBase * distanceFactor * tagFactor * VisionConstants.kMT2TrustFactor);
      double rotStdDev = Double.MAX_VALUE;

      swerveSubsystem.getSwerveDrive().addVisionMeasurement(
          visionPose,
          estimate.timestampSeconds,
          VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev));

      // Update tracking
      if (isFront) {
        lastFrontPose = visionPose;
        lastFrontAcceptTime = now;
      } else {
        lastBackPose = visionPose;
        lastBackAcceptTime = now;
      }

      totalTagCount += estimate.tagCount;

      publishCameraTelemetry(cameraTable, visionPose, estimate.tagCount,
          estimate.avgTagDist, xyStdDev, true, "", estimate.latency, poseJump);
    } else {
      publishCameraTelemetry(cameraTable, visionPose, estimate.tagCount,
          estimate.avgTagDist, 0, false, rejectReason, estimate.latency, poseJump);
    }
  }

  /** Sets both cameras to SyncInternalImu (seed mode). */
  private void enterSeedMode() {
    limelightFront.getSettings().withImuMode(ImuMode.SyncInternalImu).save();
    limelightBack.getSettings().withImuMode(ImuMode.SyncInternalImu).save();
    imuSettleCycleCount = 0;
    imuPhase = "SEEDING";
  }

  /** Sets both cameras to InternalImuExternalAssist (active mode). */
  private void enterActiveMode() {
    limelightFront.getSettings().withImuMode(ImuMode.InternalImuExternalAssist).save();
    limelightBack.getSettings().withImuMode(ImuMode.InternalImuExternalAssist).save();
    imuSettleCycleCount = 0;
    imuPhase = "SETTLING";
  }

  // --- Utility methods ---

  /** @return true if at least one camera accepted a measurement in the last 500ms. */
  public boolean isVisionHealthy() {
    double now = Timer.getFPGATimestamp();
    return (now - lastFrontAcceptTime < 0.5) || (now - lastBackAcceptTime < 0.5);
  }

  /**
   * Gets the distance to a specific AprilTag from either camera.
   * Useful for shooter interpolation.
   *
   * @param tagId The AprilTag ID to find.
   * @return Distance in meters, or -1 if tag not visible.
   */
  public double getDistanceToTag(int tagId) {
    double dist = getDistanceToTagFromEstimate(frontPoseEstimate, tagId);
    if (dist >= 0) return dist;
    return getDistanceToTagFromEstimate(backPoseEstimate, tagId);
  }

  private double getDistanceToTagFromEstimate(PoseEstimate estimate, int tagId) {
    if (!estimate.hasData) return -1;

    for (RawFiducial fiducial : estimate.rawFiducials) {
      if (fiducial.id == tagId) {
        return fiducial.distToRobot;
      }
    }
    return -1;
  }

  // --- Telemetry ---

  private void publishCameraTelemetry(NetworkTable table, Pose2d pose, int tagCount,
      double avgTagDist, double xyStdDev, boolean accepted, String rejectReason,
      double latencyMs, double poseJump) {
    if (pose != null) {
      table.getEntry(NetworkTableNames.Vision.kVisionPose)
          .setDoubleArray(new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() });
    }
    table.getEntry(NetworkTableNames.Vision.kTagCount).setDouble(tagCount);
    table.getEntry(NetworkTableNames.Vision.kAvgTagDistance).setDouble(avgTagDist);
    table.getEntry(NetworkTableNames.Vision.kXYStdDev).setDouble(xyStdDev);
    table.getEntry(NetworkTableNames.Vision.kAccepted).setBoolean(accepted);
    table.getEntry(NetworkTableNames.Vision.kRejectReason).setString(rejectReason);
    table.getEntry(NetworkTableNames.Vision.kLatencyMs).setDouble(latencyMs);
    table.getEntry(NetworkTableNames.Vision.kPoseJumpMeters).setDouble(poseJump);
  }

  private void publishSystemTelemetry() {
    visionTable.getEntry(NetworkTableNames.Vision.kTotalTagCount).setDouble(totalTagCount);
    visionTable.getEntry(NetworkTableNames.Vision.kVisionHealthy).setBoolean(isVisionHealthy());
    visionTable.getEntry(NetworkTableNames.Vision.kImuPhase).setString(imuPhase);
  }

  @Override
  public void simulationPeriodic() {}
}
