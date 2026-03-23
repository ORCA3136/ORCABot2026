package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
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
import limelight.networktables.LimelightUtils;
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
 * Each cycle:
 * <ol>
 *   <li>Feeds the robot's heading to both Limelights (required for MegaTag2)</li>
 *   <li>Gets MegaTag2 pose estimates from both cameras</li>
 *   <li>Runs a 4-stage rejection filter (stale, field boundary, yaw rate, tag distance)</li>
 *   <li>Evaluates dual-camera agreement to determine trust level</li>
 *   <li>Calculates dynamic standard deviations based on tag distance, count, and agreement</li>
 *   <li>Fuses accepted measurements into the swerve drive's Kalman filter</li>
 * </ol>
 *
 * <h2>IMU mode lifecycle</h2>
 * <ul>
 *   <li><b>Disabled:</b> SyncInternalImu — Limelight syncs its IMU to our gyro.
 *       MT1 used to seed odometry with heading-independent pose.</li>
 *   <li><b>Enabled:</b> ExternalImu — Limelight uses ONLY the heading we provide via
 *       {@code withRobotOrientation()}. MT2 used for Kalman filter fusion.</li>
 * </ul>
 */
public class VisionSubsystem extends SubsystemBase {

  /** Result of evaluating a single camera — deferred fusion. */
  private static class CameraResult {
    Pose2d pose;
    double timestamp;
    double baseXYStdDev;
    int tagCount;
    double avgTagDist;
    boolean accepted;
    boolean seeded; // true if this result triggered an odometry seed (don't fuse again)
    String rejectReason;
  }

  private final SwerveSubsystem swerveSubsystem;

  private final Limelight limelightFront;
  private final Limelight limelightBack;

  // MT2 for enabled-period Kalman fusion
  private final PoseEstimate frontPoseEstimate;
  private final PoseEstimate backPoseEstimate;

  // MT1 for disabled-period seeding (heading-independent ground truth)
  private final PoseEstimate frontPoseMT1Estimate;
  private final PoseEstimate backPoseMT1Estimate;

  // --- IMU state ---
  private boolean wasEnabled = false;
  private int imuSettleCycleCount = 0;
  private boolean imuSettled = false;

  // --- Vision tracking ---
  private boolean hasEverHadFix = false;
  private boolean loggedDisabledSeed = false;
  private double lastAcceptTime = 0;

  // Tag count tracking for combined total
  private int lastFrontTagCount = 0;
  private int lastBackTagCount = 0;

  // LED pattern state machine
  private enum LedState { OFF, SOLID, BLINK_ON, BLINK_OFF, PAUSE }
  private LedState ledState = LedState.OFF;
  private int ledCycleCounter = 0;
  private int ledBlinksRemaining = 0;
  private boolean ledIsOn = false;

  // --- Drift detection state ---
  private Pose2d lastFrontVisionPose = null;
  private Pose2d lastBackVisionPose = null;
  private double lastFrontVisionTimestamp = 0;
  private double lastBackVisionTimestamp = 0;
  private double lastFrontAvgTagDist = 0;
  private double lastBackAvgTagDist = 0;
  private int frontDriftCycleCount = 0;
  private int backDriftCycleCount = 0;
  private int hardResetCycleCount = 0;
  private int singleCameraHardResetCycleCount = 0;
  private boolean driftDetected = false;
  private double driftMagnitude = 0;
  private double cameraAgreement = -1;
  private int trustBoostCyclesRemaining = 0;
  private int hardResetCount = 0;
  private boolean dualCameraAgreed = false;
  private double lastHeadingError = 0;

  // --- Camera health state ---
  private double lastFrontDataTime = 0;
  private double lastBackDataTime = 0;
  private boolean frontStale = false;
  private boolean backStale = false;
  private int frontRebootCount = 0;
  private int backRebootCount = 0;
  private double lastFrontRebootTime = 0;
  private double lastBackRebootTime = 0;

  // --- NetworkTable entries ---
  private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
  private final NetworkTable frontTable = ntInstance.getTable(NetworkTableNames.Vision.kFrontTable);
  private final NetworkTable backTable = ntInstance.getTable(NetworkTableNames.Vision.kBackTable);
  private final NetworkTable visionTable = ntInstance.getTable(NetworkTableNames.Vision.kTable);

  // Front camera
  private final StructPublisher<Pose2d> frontPosePublisher = frontTable
      .getStructTopic(NetworkTableNames.Vision.kVisionPose, Pose2d.struct).publish();
  private final NetworkTableEntry frontTagCountEntry = frontTable.getEntry(NetworkTableNames.Vision.kTagCount);
  private final NetworkTableEntry frontAvgDistEntry = frontTable.getEntry(NetworkTableNames.Vision.kAvgTagDistance);
  private final NetworkTableEntry frontAcceptedEntry = frontTable.getEntry(NetworkTableNames.Vision.kAccepted);
  private final NetworkTableEntry frontRejectEntry = frontTable.getEntry(NetworkTableNames.Vision.kRejectReason);

  // Back camera
  private final StructPublisher<Pose2d> backPosePublisher = backTable
      .getStructTopic(NetworkTableNames.Vision.kVisionPose, Pose2d.struct).publish();
  private final NetworkTableEntry backTagCountEntry = backTable.getEntry(NetworkTableNames.Vision.kTagCount);
  private final NetworkTableEntry backAvgDistEntry = backTable.getEntry(NetworkTableNames.Vision.kAvgTagDistance);
  private final NetworkTableEntry backAcceptedEntry = backTable.getEntry(NetworkTableNames.Vision.kAccepted);
  private final NetworkTableEntry backRejectEntry = backTable.getEntry(NetworkTableNames.Vision.kRejectReason);

  // Front MT1 pose
  private final StructPublisher<Pose2d> frontPoseMT1Publisher = frontTable
      .getStructTopic(NetworkTableNames.Vision.kVisionPoseMT1, Pose2d.struct).publish();
  // Back MT1 pose
  private final StructPublisher<Pose2d> backPoseMT1Publisher = backTable
      .getStructTopic(NetworkTableNames.Vision.kVisionPoseMT1, Pose2d.struct).publish();

  // System-level
  private final NetworkTableEntry visionHealthyEntry = visionTable.getEntry(NetworkTableNames.Vision.kVisionHealthy);
  private final NetworkTableEntry imuSettledEntry = visionTable.getEntry(NetworkTableNames.Vision.kImuSettled);
  private final NetworkTableEntry odometryHeadingEntry = visionTable.getEntry(NetworkTableNames.Vision.kOdometryHeading);
  private final NetworkTableEntry pigeonRawYawEntry = visionTable.getEntry(NetworkTableNames.Vision.kPigeonRawYaw);
  private final NetworkTableEntry detectedAllianceEntry = visionTable.getEntry(NetworkTableNames.Vision.kDetectedAlliance);
  private final NetworkTableEntry firstFixStatusEntry = visionTable.getEntry(NetworkTableNames.Vision.kFirstFixStatus);
  private final NetworkTableEntry imuModeEntry = visionTable.getEntry(NetworkTableNames.Vision.kImuMode);
  private final NetworkTableEntry aprilTagReadyEntry = visionTable.getEntry(NetworkTableNames.Vision.kAprilTagReady);
  private final NetworkTableEntry totalTagCountEntry = visionTable.getEntry(NetworkTableNames.Vision.kTotalTagCount);

  // Drift detection telemetry
  private final NetworkTableEntry driftDetectedEntry = visionTable.getEntry(NetworkTableNames.Vision.kDriftDetected);
  private final NetworkTableEntry driftMagnitudeEntry = visionTable.getEntry(NetworkTableNames.Vision.kDriftMagnitude);
  private final NetworkTableEntry cameraAgreementEntry = visionTable.getEntry(NetworkTableNames.Vision.kCameraAgreement);
  private final NetworkTableEntry trustBoostedEntry = visionTable.getEntry(NetworkTableNames.Vision.kTrustBoosted);
  private final NetworkTableEntry hardResetCountEntry = visionTable.getEntry(NetworkTableNames.Vision.kHardResetCount);
  private final NetworkTableEntry dualCameraAgreedEntry = visionTable.getEntry(NetworkTableNames.Vision.kDualCameraAgreed);
  private final NetworkTableEntry headingErrorEntry = visionTable.getEntry(NetworkTableNames.Vision.kHeadingError);

  // Per-camera health telemetry
  private final NetworkTableEntry frontStaleEntry = frontTable.getEntry(NetworkTableNames.Vision.kCameraStale);
  private final NetworkTableEntry backStaleEntry = backTable.getEntry(NetworkTableNames.Vision.kCameraStale);
  private final NetworkTableEntry frontRebootCountEntry = frontTable.getEntry(NetworkTableNames.Vision.kRebootCount);
  private final NetworkTableEntry backRebootCountEntry = backTable.getEntry(NetworkTableNames.Vision.kRebootCount);

  // Track current IMU mode label for telemetry
  private String currentImuModeLabel = "SyncInternalImu";

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;

    limelightFront = new Limelight(VisionConstants.kLimelightFrontName);
    limelightBack = new Limelight(VisionConstants.kLimelightBackName);

    limelightFront.getSettings()
        .withLimelightLEDMode(LEDMode.ForceOff)
        .withImuMode(ImuMode.SyncInternalImu)
        .save();

    limelightBack.getSettings()
        .withLimelightLEDMode(LEDMode.ForceOff)
        .withImuMode(ImuMode.SyncInternalImu)
        .save();

    frontPoseEstimate = new PoseEstimate(limelightFront, "botpose_orb_wpiblue", true);
    backPoseEstimate = new PoseEstimate(limelightBack, "botpose_orb_wpiblue", true);
    frontPoseMT1Estimate = new PoseEstimate(limelightFront, "botpose_wpiblue", true);
    backPoseMT1Estimate = new PoseEstimate(limelightBack, "botpose_wpiblue", true);

    // Initialize health timestamps so cameras aren't immediately flagged stale
    double startTime = Timer.getFPGATimestamp();
    lastFrontDataTime = startTime;
    lastBackDataTime = startTime;
  }

  @Override
  public void periodic() {
    boolean isEnabled = DriverStation.isEnabled();

    // --- IMU mode transitions ---
    if (!isEnabled && wasEnabled) {
      enterSeedMode();
      wasEnabled = false;
    }
    if (isEnabled && !wasEnabled) {
      enterActiveMode();
      wasEnabled = true;
    }

    // --- Feed heading to both cameras every frame ---
    // Use raw Pigeon2 rotation (not odometry heading) to avoid circular dependency
    Rotation3d pigeonRotation = swerveSubsystem.getGyroRotation3d();
    Rotation3d feedRotation = new Rotation3d(
        pigeonRotation.getMeasureX(),
        pigeonRotation.getMeasureY(),
        pigeonRotation.getMeasureZ());

    Orientation3d orientation = new Orientation3d(
        feedRotation,
        new AngularVelocity3d(
            DegreesPerSecond.of(swerveSubsystem.getPigeon2RollRateDegPerSec()),
            DegreesPerSecond.of(swerveSubsystem.getPigeon2PitchRateDegPerSec()),
            DegreesPerSecond.of(swerveSubsystem.getPigeon2YawRateDegPerSec())));

    limelightFront.getSettings().withRobotOrientation(orientation);
    limelightBack.getSettings().withRobotOrientation(orientation);

    // --- Wait for IMU to settle after mode switch ---
    if (!imuSettled) {
      imuSettleCycleCount++;
      if (imuSettleCycleCount >= VisionConstants.kImuSettleCycles) {
        imuSettled = true;
      }
      publishSystemTelemetry();
      return;
    }

    // --- Evaluate both cameras (deferred fusion) ---
    CameraResult frontResult = evaluateCamera(frontPoseEstimate, true);
    CameraResult backResult = evaluateCamera(backPoseEstimate, false);

    // --- Decide trust levels and fuse ---
    fuseVisionResults(frontResult, backResult);

    // --- Drift detection (only when enabled and we have a fix) ---
    if (isEnabled && hasEverHadFix) {
      updateDriftDetection();
    }

    // --- Camera health watchdog ---
    updateCameraHealth();

    // --- LED feedback while disabled ---
    int totalTags = lastFrontTagCount + lastBackTagCount;
    if (!isEnabled) {
      updateDisabledLeds(totalTags);
    }

    publishSystemTelemetry();
  }

  /**
   * Evaluates a single camera's vision data. Returns a CameraResult without fusing —
   * fusion is deferred to fuseVisionResults() so we can compare both cameras first.
   */
  private CameraResult evaluateCamera(PoseEstimate poseEstimateObj, boolean isFront) {
    CameraResult result = new CameraResult();
    result.accepted = false;
    result.seeded = false;
    result.rejectReason = "no_data";
    result.tagCount = 0;
    result.avgTagDist = 0;

    StructPublisher<Pose2d> posePublisher = isFront ? frontPosePublisher : backPosePublisher;
    NetworkTableEntry tagCountEntry = isFront ? frontTagCountEntry : backTagCountEntry;
    NetworkTableEntry avgDistEntry = isFront ? frontAvgDistEntry : backAvgDistEntry;
    NetworkTableEntry acceptedEntry = isFront ? frontAcceptedEntry : backAcceptedEntry;
    NetworkTableEntry rejectEntry = isFront ? frontRejectEntry : backRejectEntry;

    Optional<PoseEstimate> estimateOpt = poseEstimateObj.getPoseEstimate();

    if (estimateOpt.isEmpty() || !estimateOpt.get().hasData) {
      if (isFront) lastFrontTagCount = 0; else lastBackTagCount = 0;
      publishCameraTelemetry(posePublisher, tagCountEntry, avgDistEntry, acceptedEntry, rejectEntry,
          null, 0, 0, false, "no_data");
      return result;
    }

    PoseEstimate estimate = estimateOpt.get();
    if (isFront) lastFrontTagCount = estimate.tagCount; else lastBackTagCount = estimate.tagCount;
    Pose2d visionPose = estimate.pose.toPose2d();
    double now = Timer.getFPGATimestamp();

    // Record vision pose for drift detection, and update data time for staleness tracking
    if (isFront) {
      lastFrontVisionPose = visionPose;
      lastFrontVisionTimestamp = now;
      lastFrontDataTime = now;
      lastFrontAvgTagDist = estimate.avgTagDist;
    } else {
      lastBackVisionPose = visionPose;
      lastBackVisionTimestamp = now;
      lastBackDataTime = now;
      lastBackAvgTagDist = estimate.avgTagDist;
    }

    // Publish MT1 pose alongside MT2 for comparison
    StructPublisher<Pose2d> mt1Publisher = isFront ? frontPoseMT1Publisher : backPoseMT1Publisher;
    PoseEstimate mt1Obj = isFront ? frontPoseMT1Estimate : backPoseMT1Estimate;
    Optional<PoseEstimate> mt1Opt = mt1Obj.getPoseEstimate();
    if (mt1Opt.isPresent() && mt1Opt.get().hasData && mt1Opt.get().tagCount > 0) {
      Pose2d mt1Pose = mt1Opt.get().pose.toPose2d();
      mt1Publisher.set(mt1Pose);

      // Log large MT1 vs MT2 heading divergence for post-match analysis
      double headingDiff = Math.abs(mt1Pose.getRotation().getDegrees() - visionPose.getRotation().getDegrees());
      if (headingDiff > 180) headingDiff = 360 - headingDiff;
      if (headingDiff > 10) {
        DataLogManager.log("Vision: MT1/MT2 heading divergence "
            + String.format("%.1f", headingDiff) + "deg on " + (isFront ? "front" : "back"));
      }
    }

    // --- 4-stage rejection ---
    String rejectReason = getRejectReason(estimate, visionPose, now);

    if (!rejectReason.isEmpty()) {
      result.rejectReason = rejectReason;
      publishCameraTelemetry(posePublisher, tagCountEntry, avgDistEntry, acceptedEntry, rejectEntry,
          visionPose, estimate.tagCount, estimate.avgTagDist, false, rejectReason);
      return result;
    }

    // --- Calculate dynamic XY std devs ---
    double distanceFactor = estimate.avgTagDist * estimate.avgTagDist;
    double tagFactor = 1.0 / Math.max(1, estimate.tagCount);
    double singleTagPenalty = (estimate.tagCount == 1) ? VisionConstants.kSingleTagPenalty : 1.0;
    double xyStdDev = Math.max(VisionConstants.kMinXYStdDev,
        VisionConstants.kXYStdDevBase * distanceFactor * tagFactor * singleTagPenalty);

    boolean highConfidence = estimate.tagCount >= VisionConstants.kSeedMinTagCount
        && estimate.avgTagDist < VisionConstants.kSeedMaxDistM;

    if (!hasEverHadFix || (!DriverStation.isEnabled() && highConfidence)) {
      // --- Seed odometry from MT1 (heading-independent ground truth) ---
      Pose2d seedPose;
      if (mt1Opt.isPresent() && mt1Opt.get().hasData && mt1Opt.get().tagCount > 0) {
        seedPose = mt1Opt.get().pose.toPose2d();
      } else {
        seedPose = visionPose;
      }

      swerveSubsystem.resetOdometry(seedPose);
      imuSettleCycleCount = 0;
      imuSettled = false;

      if (!hasEverHadFix) {
        DataLogManager.log("Vision: first fix — reset odometry to " + seedPose);
        hasEverHadFix = true;
      } else if (!loggedDisabledSeed) {
        DataLogManager.log("Vision: disabled seed — reset odometry to " + seedPose);
        loggedDisabledSeed = true;
      }

      result.seeded = true;
      result.accepted = true;
      result.rejectReason = "";
      lastAcceptTime = now;
      publishCameraTelemetry(posePublisher, tagCountEntry, avgDistEntry, acceptedEntry, rejectEntry,
          visionPose, estimate.tagCount, estimate.avgTagDist, true, "");
      return result;
    }

    // Accepted for fusion — populate result
    result.pose = visionPose;
    result.timestamp = estimate.timestampSeconds;
    result.baseXYStdDev = xyStdDev;
    result.tagCount = estimate.tagCount;
    result.avgTagDist = estimate.avgTagDist;
    result.accepted = true;
    result.rejectReason = "";

    lastAcceptTime = now;
    publishCameraTelemetry(posePublisher, tagCountEntry, avgDistEntry, acceptedEntry, rejectEntry,
        visionPose, estimate.tagCount, estimate.avgTagDist, true, "");
    return result;
  }

  /**
   * Decides how to fuse vision results based on dual-camera agreement and single-camera conditions.
   * Called after both cameras are evaluated so we can compare their results.
   */
  private void fuseVisionResults(CameraResult front, CameraResult back) {
    boolean frontReady = front.accepted && !front.seeded && front.pose != null;
    boolean backReady = back.accepted && !back.seeded && back.pose != null;
    dualCameraAgreed = false;

    if (!frontReady && !backReady) {
      return; // Nothing to fuse
    }

    if (frontReady && backReady) {
      // Both cameras accepted — check agreement
      double agreement = front.pose.getTranslation().getDistance(back.pose.getTranslation());

      if (agreement < VisionConstants.kDualCameraAgreementThresholdM) {
        // Both cameras agree — high trust, use tight std devs
        dualCameraAgreed = true;
        double stdDev = getEffectiveXYStdDev(VisionConstants.kDualCameraAgreedStdDev);
        swerveSubsystem.addVisionMeasurement(
            front.pose, front.timestamp, VecBuilder.fill(stdDev, stdDev, 9999.0));
        swerveSubsystem.addVisionMeasurement(
            back.pose, back.timestamp, VecBuilder.fill(stdDev, stdDev, 9999.0));
      } else {
        // Both accepted but disagree — use normal dynamic std devs
        double frontStd = getEffectiveXYStdDev(front.baseXYStdDev);
        double backStd = getEffectiveXYStdDev(back.baseXYStdDev);
        swerveSubsystem.addVisionMeasurement(
            front.pose, front.timestamp, VecBuilder.fill(frontStd, frontStd, 9999.0));
        swerveSubsystem.addVisionMeasurement(
            back.pose, back.timestamp, VecBuilder.fill(backStd, backStd, 9999.0));
      }
    } else {
      // Only one camera accepted — check for single-camera boost
      CameraResult single = frontReady ? front : back;
      boolean otherCameraStale = frontReady ? backStale : frontStale;

      double stdDev;
      if (otherCameraStale
          && single.tagCount >= VisionConstants.kSingleCameraBoostMinTags
          && single.avgTagDist < VisionConstants.kSingleCameraBoostMaxDistM) {
        // Other camera is confirmed stale, single camera has good multi-tag data
        stdDev = getEffectiveXYStdDev(VisionConstants.kSingleCameraBoostStdDev);
      } else {
        stdDev = getEffectiveXYStdDev(single.baseXYStdDev);
      }

      swerveSubsystem.addVisionMeasurement(
          single.pose, single.timestamp, VecBuilder.fill(stdDev, stdDev, 9999.0));
    }
  }

  /**
   * 6-stage rejection filter. Returns empty string if accepted.
   */
  private String getRejectReason(PoseEstimate estimate, Pose2d visionPose, double now) {
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

    // 3. High yaw rate (MegaTag2 is unreliable during fast rotation)
    double yawRate = Math.abs(swerveSubsystem.getPigeon2YawRateDegPerSec());
    if (yawRate > VisionConstants.kMaxYawRateDegPerSec) {
      return "high_yaw_rate";
    }

    // 4. Max tag distance — tags beyond 6m are unreliable
    if (estimate.avgTagDist > VisionConstants.kMaxTagDistanceM) {
      return "too_far";
    }

    // 5. Single-tag at distance — high 180° ambiguity risk
    if (estimate.tagCount == 1 && estimate.avgTagDist > VisionConstants.kSingleTagMaxDistanceM) {
      return "single_tag_too_far";
    }

    // 6. Heading cross-check — catch 180° MT2 flips
    double visionHeadingDeg = visionPose.getRotation().getDegrees();
    double gyroHeadingDeg = swerveSubsystem.getPigeonRawYawDeg();
    double headingError = Math.abs(visionHeadingDeg - gyroHeadingDeg);
    if (headingError > 180) headingError = 360 - headingError;
    lastHeadingError = headingError;
    if (headingError > VisionConstants.kMaxHeadingErrorDeg) {
      return "heading_flip";
    }

    return "";
  }

  // --- IMU mode transitions ---

  private void enterSeedMode() {
    limelightFront.getSettings().withImuMode(ImuMode.SyncInternalImu).save();
    limelightBack.getSettings().withImuMode(ImuMode.SyncInternalImu).save();
    imuSettleCycleCount = 0;
    imuSettled = false;
    currentImuModeLabel = "SyncInternalImu";
    loggedDisabledSeed = false;
    DataLogManager.log("Vision: entering seed mode (SyncInternalImu)");
  }

  private void enterActiveMode() {
    limelightFront.getSettings()
        .withImuMode(ImuMode.ExternalImu)
        .withLimelightLEDMode(LEDMode.ForceOff)
        .save();
    limelightBack.getSettings()
        .withImuMode(ImuMode.ExternalImu)
        .withLimelightLEDMode(LEDMode.ForceOff)
        .save();
    ledIsOn = false;
    ledState = LedState.OFF;
    imuSettleCycleCount = 0;
    imuSettled = false;
    currentImuModeLabel = "ExternalImu";
    DataLogManager.log("Vision: entering active mode (ExternalImu), settling for "
        + VisionConstants.kImuSettleCycles + " cycles");
  }

  // --- LED feedback ---

  private void setLedOn(boolean on) {
    if (on == ledIsOn) return;
    LEDMode mode = on ? LEDMode.ForceOn : LEDMode.ForceOff;
    limelightFront.getSettings().withLimelightLEDMode(mode).save();
    limelightBack.getSettings().withLimelightLEDMode(mode).save();
    ledIsOn = on;
  }

  private void updateDisabledLeds(int totalTags) {
    if (totalTags == 0 || !hasEverHadFix) {
      setLedOn(false);
      ledState = LedState.OFF;
      return;
    }

    ledCycleCounter++;

    switch (ledState) {
      case OFF:
        setLedOn(true);
        ledState = LedState.SOLID;
        ledCycleCounter = 0;
        break;

      case SOLID:
        if (ledCycleCounter >= 250) {
          ledBlinksRemaining = totalTags;
          ledState = LedState.BLINK_ON;
          ledCycleCounter = 0;
          setLedOn(true);
        }
        break;

      case BLINK_ON:
        if (ledCycleCounter >= 15) {
          setLedOn(false);
          ledState = LedState.BLINK_OFF;
          ledCycleCounter = 0;
        }
        break;

      case BLINK_OFF:
        if (ledCycleCounter >= 15) {
          ledBlinksRemaining--;
          if (ledBlinksRemaining > 0) {
            setLedOn(true);
            ledState = LedState.BLINK_ON;
          } else {
            ledState = LedState.PAUSE;
          }
          ledCycleCounter = 0;
        }
        break;

      case PAUSE:
        if (ledCycleCounter >= 150) {
          setLedOn(true);
          ledState = LedState.SOLID;
          ledCycleCounter = 0;
        }
        break;
    }
  }

  // --- Drift detection & camera health ---

  /** Returns boosted std dev if trust boost is active, otherwise returns the base value. */
  private double getEffectiveXYStdDev(double baseStdDev) {
    if (trustBoostCyclesRemaining > 0) {
      return Math.min(baseStdDev, VisionConstants.kBoostedXYStdDev);
    }
    return baseStdDev;
  }

  /** Compares vision poses to odometry and detects drift, triggers trust boost or hard reset. */
  private void updateDriftDetection() {
    Pose2d odometryPose = swerveSubsystem.getPose();
    double now = Timer.getFPGATimestamp();

    // Check freshness of each camera's last vision pose
    boolean frontFresh = lastFrontVisionPose != null
        && (now - lastFrontVisionTimestamp) < VisionConstants.kDriftPoseFreshnessMaxSec;
    boolean backFresh = lastBackVisionPose != null
        && (now - lastBackVisionTimestamp) < VisionConstants.kDriftPoseFreshnessMaxSec;

    // Compute per-camera drift magnitude
    double frontDrift = frontFresh
        ? lastFrontVisionPose.getTranslation().getDistance(odometryPose.getTranslation()) : 0;
    double backDrift = backFresh
        ? lastBackVisionPose.getTranslation().getDistance(odometryPose.getTranslation()) : 0;
    driftMagnitude = Math.max(frontDrift, backDrift);

    // Increment or reset per-camera drift counters
    if (frontFresh && frontDrift > VisionConstants.kDriftDetectionThresholdM) {
      frontDriftCycleCount++;
    } else {
      frontDriftCycleCount = 0;
    }
    if (backFresh && backDrift > VisionConstants.kDriftDetectionThresholdM) {
      backDriftCycleCount++;
    } else {
      backDriftCycleCount = 0;
    }

    driftDetected = frontDriftCycleCount >= VisionConstants.kDriftConfirmCycles
        || backDriftCycleCount >= VisionConstants.kDriftConfirmCycles;

    // Compute camera agreement (only if both fresh)
    if (frontFresh && backFresh) {
      cameraAgreement = lastFrontVisionPose.getTranslation()
          .getDistance(lastBackVisionPose.getTranslation());
    } else {
      cameraAgreement = -1;
    }

    // Trust boost: both cameras fresh, agreeing, and drift detected
    if (driftDetected && frontFresh && backFresh
        && cameraAgreement >= 0 && cameraAgreement < VisionConstants.kCameraAgreementMaxM) {
      trustBoostCyclesRemaining = VisionConstants.kBoostDurationCycles;
    }

    // --- Hard reset: dual-camera path ---
    if (frontFresh && backFresh
        && frontDrift > VisionConstants.kHardResetThresholdM
        && backDrift > VisionConstants.kHardResetThresholdM
        && cameraAgreement >= 0 && cameraAgreement < VisionConstants.kCameraAgreementMaxM) {
      hardResetCycleCount++;
      if (hardResetCycleCount >= VisionConstants.kHardResetConfirmCycles) {
        // Average the two camera poses for the reset
        Translation2d avgTranslation = lastFrontVisionPose.getTranslation()
            .plus(lastBackVisionPose.getTranslation()).div(2.0);
        Rotation2d resetRotation = (lastFrontTagCount >= lastBackTagCount)
            ? lastFrontVisionPose.getRotation() : lastBackVisionPose.getRotation();
        Pose2d resetPose = new Pose2d(avgTranslation, resetRotation);

        swerveSubsystem.resetOdometry(resetPose);
        hardResetCount++;
        hardResetCycleCount = 0;
        singleCameraHardResetCycleCount = 0;
        frontDriftCycleCount = 0;
        backDriftCycleCount = 0;
        driftDetected = false;
        DataLogManager.log("Vision: HARD RESET (dual-cam) odometry to " + resetPose
            + " (drift=" + String.format("%.2f", driftMagnitude)
            + "m, agreement=" + String.format("%.2f", cameraAgreement) + "m)");
      }
    } else {
      hardResetCycleCount = 0;
    }

    // --- Hard reset: single-camera path ---
    // Only if one camera is stale (confirmed dead) and the other has high-confidence data
    boolean frontOnlyWithDrift = frontFresh && backStale
        && frontDrift > VisionConstants.kHardResetThresholdM
        && lastFrontTagCount >= VisionConstants.kSingleCameraHardResetMinTags
        && lastFrontVisionPose != null;
    boolean backOnlyWithDrift = backFresh && frontStale
        && backDrift > VisionConstants.kHardResetThresholdM
        && lastBackTagCount >= VisionConstants.kSingleCameraHardResetMinTags
        && lastBackVisionPose != null;

    if (frontOnlyWithDrift || backOnlyWithDrift) {
      boolean isFrontActive = frontOnlyWithDrift;
      double activeAvgTagDist = isFrontActive ? lastFrontAvgTagDist : lastBackAvgTagDist;

      if (activeAvgTagDist < VisionConstants.kSingleCameraHardResetMaxDistM) {
        singleCameraHardResetCycleCount++;
        if (singleCameraHardResetCycleCount >= VisionConstants.kSingleCameraHardResetCycles) {
          Pose2d resetPose = isFrontActive ? lastFrontVisionPose : lastBackVisionPose;
          swerveSubsystem.resetOdometry(resetPose);
          hardResetCount++;
          singleCameraHardResetCycleCount = 0;
          frontDriftCycleCount = 0;
          backDriftCycleCount = 0;
          driftDetected = false;
          DataLogManager.log("Vision: HARD RESET (single-cam "
              + (isFrontActive ? "front" : "back") + ") odometry to " + resetPose
              + " (drift=" + String.format("%.2f", driftMagnitude) + "m)");
        }
      } else {
        singleCameraHardResetCycleCount = 0;
      }
    } else {
      singleCameraHardResetCycleCount = 0;
    }

    // Decrement trust boost counter
    if (trustBoostCyclesRemaining > 0) {
      trustBoostCyclesRemaining--;
    }
  }

  /** Monitors camera data freshness and triggers HTTP reboot if a camera goes silent. */
  private void updateCameraHealth() {
    double now = Timer.getFPGATimestamp();

    // Front camera
    double frontAge = now - lastFrontDataTime;
    frontStale = frontAge > VisionConstants.kCameraStaleThresholdSec;
    if (frontAge > VisionConstants.kCameraRebootThresholdSec
        && (now - lastFrontRebootTime) > VisionConstants.kRebootCooldownSec) {
      rebootCameraAsync(true);
      lastFrontRebootTime = now;
      frontRebootCount++;
    }

    // Back camera
    double backAge = now - lastBackDataTime;
    backStale = backAge > VisionConstants.kCameraStaleThresholdSec;
    if (backAge > VisionConstants.kCameraRebootThresholdSec
        && (now - lastBackRebootTime) > VisionConstants.kRebootCooldownSec) {
      rebootCameraAsync(false);
      lastBackRebootTime = now;
      backRebootCount++;
    }
  }

  /** Sends an async HTTP reboot request to a Limelight. Fire-and-forget, does not block. */
  private void rebootCameraAsync(boolean isFront) {
    String cameraName = isFront ? VisionConstants.kLimelightFrontName : VisionConstants.kLimelightBackName;
    DataLogManager.log("Vision: attempting HTTP reboot of " + cameraName);

    CompletableFuture.supplyAsync(() -> {
      try {
        URL url = LimelightUtils.getLimelightURLString(cameraName, "reboot");
        if (url == null) {
          DataLogManager.log("Vision: reboot of " + cameraName + " failed — bad URL");
          return false;
        }
        HttpURLConnection connection = (HttpURLConnection) url.openConnection();
        connection.setRequestMethod("POST");
        connection.setConnectTimeout(VisionConstants.kRebootHttpTimeoutMs);
        connection.setReadTimeout(VisionConstants.kRebootHttpTimeoutMs);
        int responseCode = connection.getResponseCode();
        connection.disconnect();
        if (responseCode == 200) {
          DataLogManager.log("Vision: reboot of " + cameraName + " succeeded (HTTP 200)");
          return true;
        } else {
          DataLogManager.log("Vision: reboot of " + cameraName + " got HTTP " + responseCode);
        }
      } catch (IOException e) {
        DataLogManager.log("Vision: reboot of " + cameraName + " failed: " + e.getMessage());
      }
      return false;
    });
  }

  // --- Utility methods ---

  /** @return true if at least one camera accepted a measurement recently. */
  public boolean isVisionHealthy() {
    return (Timer.getFPGATimestamp() - lastAcceptTime) < VisionConstants.kVisionHealthyTimeoutSec;
  }

  /**
   * Gets the distance to a specific AprilTag from either camera.
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
      NetworkTableEntry acceptedNt, NetworkTableEntry rejectNt,
      Pose2d pose, int tagCount, double avgTagDist, boolean accepted, String rejectReason) {
    if (pose != null) {
      posePub.set(pose);
    }
    tagCountNt.setDouble(tagCount);
    avgDistNt.setDouble(avgTagDist);
    acceptedNt.setBoolean(accepted);
    rejectNt.setString(rejectReason);
  }

  private void publishSystemTelemetry() {
    visionHealthyEntry.setBoolean(isVisionHealthy());
    imuSettledEntry.setBoolean(imuSettled);
    odometryHeadingEntry.setDouble(swerveSubsystem.getHeading().getDegrees());
    pigeonRawYawEntry.setDouble(swerveSubsystem.getPigeonRawYawDeg());
    Optional<DriverStation.Alliance> rawAlliance = DriverStation.getAlliance();
    detectedAllianceEntry.setString(rawAlliance.isPresent() ? rawAlliance.get().name() : "NOT_SET");
    firstFixStatusEntry.setBoolean(hasEverHadFix);
    imuModeEntry.setString(currentImuModeLabel);
    aprilTagReadyEntry.setBoolean(hasEverHadFix && (lastFrontTagCount + lastBackTagCount) > 0);
    totalTagCountEntry.setDouble(lastFrontTagCount + lastBackTagCount);

    // Drift detection telemetry
    driftDetectedEntry.setBoolean(driftDetected);
    driftMagnitudeEntry.setDouble(driftMagnitude);
    cameraAgreementEntry.setDouble(cameraAgreement);
    trustBoostedEntry.setBoolean(trustBoostCyclesRemaining > 0);
    hardResetCountEntry.setDouble(hardResetCount);
    dualCameraAgreedEntry.setBoolean(dualCameraAgreed);
    headingErrorEntry.setDouble(lastHeadingError);

    // Camera health telemetry
    frontStaleEntry.setBoolean(frontStale);
    backStaleEntry.setBoolean(backStale);
    frontRebootCountEntry.setDouble(frontRebootCount);
    backRebootCountEntry.setDouble(backRebootCount);
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
