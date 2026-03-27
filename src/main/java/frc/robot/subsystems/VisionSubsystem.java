package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    long[] tagIds = new long[0];
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
  private double lastMT1HeadingDeg = Double.NaN;
  private int rejectCountMT1Flip = 0;
  private int consecutiveMT1FlipCount = 0;
  private int mt1HeadingRecoveryCount = 0;

  // --- Camera health state ---
  private double lastFrontDataTime = 0;
  private double lastBackDataTime = 0;
  private boolean frontStale = false;
  private boolean backStale = false;

  // --- Rate-limiting for log messages ---
  private boolean wasPitchHigh = false;
  private boolean wasHeadingErrorHigh = false;

  // --- Rejection counters ---
  private int rejectCountStale = 0;
  private int rejectCountOutOfField = 0;
  private int rejectCountYawRate = 0;
  private int rejectCountTooFar = 0;
  private int rejectCountSingleTagFar = 0;
  private int rejectCountHeadingFlip = 0;
  private int rejectCountNoData = 0;
  private int acceptCount = 0;

  // --- Field2d for AdvantageScope visualization ---
  private final Field2d visionField = new Field2d();
  private final FieldObject2d fieldOdometry;
  private final FieldObject2d fieldFrontVision;
  private final FieldObject2d fieldBackVision;

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
  // Removed: pigeonRawYaw, detectedAlliance, firstFixStatus, imuMode, aprilTagReady
  // — these are logged as string events on state change instead of per-cycle NT entries
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
  // Removed: frontStaleEntry, backStaleEntry — development-only diagnostic

  // --- Enhanced logging publishers ---

  // Per-camera: fused std dev, latency, tag IDs
  private final NetworkTableEntry frontFusedStdDevEntry = frontTable.getEntry(NetworkTableNames.Vision.kFusedStdDevXY);
  private final NetworkTableEntry backFusedStdDevEntry = backTable.getEntry(NetworkTableNames.Vision.kFusedStdDevXY);
  // Removed: frontLatencyEntry, backLatencyEntry — development-only diagnostic
  private final IntegerArrayPublisher frontTagIdsPub = frontTable
      .getIntegerArrayTopic(NetworkTableNames.Vision.kTagIDs).publish();
  private final IntegerArrayPublisher backTagIdsPub = backTable
      .getIntegerArrayTopic(NetworkTableNames.Vision.kTagIDs).publish();

  // System-level: fusion mode, pre-fusion pose, vision delta
  private final NetworkTableEntry fusionModeEntry = visionTable.getEntry(NetworkTableNames.Vision.kFusionMode);
  private final StructPublisher<Pose2d> preFusionPosePub = visionTable
      .getStructTopic(NetworkTableNames.Vision.kPreFusionPose, Pose2d.struct).publish();
  private final NetworkTableEntry visionDeltaEntry = visionTable.getEntry(NetworkTableNames.Vision.kVisionDeltaM);

  // Rejection counters
  private final NetworkTable countsTable = ntInstance.getTable(NetworkTableNames.Vision.kCountsTable);
  private final NetworkTableEntry countAcceptedEntry = countsTable.getEntry(NetworkTableNames.Vision.kCountAccepted);
  private final NetworkTableEntry countStaleEntry = countsTable.getEntry(NetworkTableNames.Vision.kCountStale);
  private final NetworkTableEntry countOutOfFieldEntry = countsTable.getEntry(NetworkTableNames.Vision.kCountOutOfField);
  private final NetworkTableEntry countYawRateEntry = countsTable.getEntry(NetworkTableNames.Vision.kCountYawRate);
  private final NetworkTableEntry countTooFarEntry = countsTable.getEntry(NetworkTableNames.Vision.kCountTooFar);
  private final NetworkTableEntry countSingleTagFarEntry = countsTable.getEntry(NetworkTableNames.Vision.kCountSingleTagFar);
  private final NetworkTableEntry countHeadingFlipEntry = countsTable.getEntry(NetworkTableNames.Vision.kCountHeadingFlip);
  private final NetworkTableEntry countMT1FlipEntry = countsTable.getEntry("MT1Flip");
  private final NetworkTableEntry mt1RecoveryCountEntry = countsTable.getEntry("MT1RecoveryCount");
  private final NetworkTableEntry countMT1FlipConsecutiveEntry = countsTable.getEntry("MT1FlipConsecutive");
  private final NetworkTableEntry countNoDataEntry = countsTable.getEntry(NetworkTableNames.Vision.kCountNoData);

  // Limelight IMU + pitch monitoring (read LL IMU directly from NetworkTables)
  private final NetworkTableEntry frontLLImuEntry = ntInstance.getTable(VisionConstants.kLimelightFrontName).getEntry("imu");
  private final NetworkTableEntry backLLImuEntry = ntInstance.getTable(VisionConstants.kLimelightBackName).getEntry("imu");
  private final NetworkTableEntry frontLLImuYawEntry = visionTable.getEntry("FrontLLInternalYaw");
  private final NetworkTableEntry backLLImuYawEntry = visionTable.getEntry("BackLLInternalYaw");
  private final NetworkTableEntry pitchDegEntry = visionTable.getEntry("PigeonPitchDeg");

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

    // Field2d for AdvantageScope — overlay odometry + both vision poses
    fieldOdometry = visionField.getObject("Odometry");
    fieldFrontVision = visionField.getObject("FrontVision");
    fieldBackVision = visionField.getObject("BackVision");
    SmartDashboard.putData("VisionField", visionField);
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
    // Pitch/roll from Pigeon2, yaw from odometry heading (which includes vision seed offset)
    Rotation3d pigeonRotation = swerveSubsystem.getGyroRotation3d();
    Rotation3d feedRotation = new Rotation3d(
        pigeonRotation.getMeasureX(),
        pigeonRotation.getMeasureY(),
        swerveSubsystem.getHeading().getMeasure());

    Orientation3d orientation = new Orientation3d(
        feedRotation,
        new AngularVelocity3d(
            DegreesPerSecond.of(swerveSubsystem.getRollRateDegPerSec()),
            DegreesPerSecond.of(swerveSubsystem.getPitchRateDegPerSec()),
            DegreesPerSecond.of(swerveSubsystem.getYawRateDegPerSec())));

    limelightFront.getSettings().withRobotOrientation(orientation);
    limelightBack.getSettings().withRobotOrientation(orientation);

    // --- Limelight IMU + pitch logging for 3-way gyro analysis ---
    // LL IMU NT array: [robotYaw, roll, pitch, internalYaw, rollRate, pitchRate, yawRate, accelX, accelY, accelZ]
    double pitchDeg = swerveSubsystem.getRawPitchDeg();
    double[] emptyImu = new double[0];
    double[] frontImu = frontLLImuEntry.getDoubleArray(emptyImu);
    double[] backImu = backLLImuEntry.getDoubleArray(emptyImu);
    double frontLLInternalYaw = frontImu.length > 3 ? frontImu[3] : Double.NaN;
    double backLLInternalYaw = backImu.length > 3 ? backImu[3] : Double.NaN;
    double frontLLRobotYaw = frontImu.length > 0 ? frontImu[0] : Double.NaN;
    double backLLRobotYaw = backImu.length > 0 ? backImu[0] : Double.NaN;
    double pigeonYaw = swerveSubsystem.getRawYawDeg();

    frontLLImuYawEntry.setDouble(frontLLInternalYaw);
    backLLImuYawEntry.setDouble(backLLInternalYaw);
    pitchDegEntry.setDouble(pitchDeg);

    boolean pitchHigh = Math.abs(pitchDeg) > 10.0;
    if (pitchHigh && !wasPitchHigh) {
      DataLogManager.log("[PITCH] entered high: " + String.format("%.1f", pitchDeg) + "°"
          + " pigeonYaw=" + String.format("%.1f", pigeonYaw)
          + " frontLL_internal=" + String.format("%.1f", frontLLInternalYaw)
          + " backLL_internal=" + String.format("%.1f", backLLInternalYaw)
          + " frontLL_robot=" + String.format("%.1f", frontLLRobotYaw)
          + " backLL_robot=" + String.format("%.1f", backLLRobotYaw));
    }
    wasPitchHigh = pitchHigh;

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
      rejectCountNoData++;
      publishCameraTelemetry(posePublisher, tagCountEntry, avgDistEntry, acceptedEntry, rejectEntry,
          null, 0, 0, false, "no_data");
      return result;
    }

    PoseEstimate estimate = estimateOpt.get();
    if (isFront) lastFrontTagCount = estimate.tagCount; else lastBackTagCount = estimate.tagCount;
    Pose2d visionPose = estimate.pose.toPose2d();
    double now = Timer.getFPGATimestamp();

    // Extract tag IDs and compute measurement latency
    long[] tagIds = new long[estimate.rawFiducials.length];
    for (int i = 0; i < estimate.rawFiducials.length; i++) {
      tagIds[i] = estimate.rawFiducials[i].id;
    }
    result.tagIds = tagIds;

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
      lastMT1HeadingDeg = mt1Pose.getRotation().getDegrees();
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

    // Secondary MT1 heading guard — catches 180° flips that the MT2-vs-odometry check misses
    if ((rejectReason.isEmpty() || rejectReason.equals("heading_flip")) && !Double.isNaN(lastMT1HeadingDeg) && estimate.tagCount > 0) {
      double mt1Error = Math.abs(visionPose.getRotation().getDegrees() - lastMT1HeadingDeg);
      if (mt1Error > 180) mt1Error = 360 - mt1Error;
      if (mt1Error > 120.0) {
        rejectReason = "mt1_flip";
        boolean pitchIsLevel = Math.abs(swerveSubsystem.getRawPitchDeg()) < VisionConstants.kPitchGateThresholdDeg;
        if (pitchIsLevel) {
          consecutiveMT1FlipCount++;
        }

        DataLogManager.log("[VISION-MT1-FLIP] MT2 hdg=" + String.format("%.1f", visionPose.getRotation().getDegrees())
            + " MT1 hdg=" + String.format("%.1f", lastMT1HeadingDeg)
            + " error=" + String.format("%.1f", mt1Error) + "°"
            + " consecutive=" + consecutiveMT1FlipCount
            + " pitch=" + String.format("%.1f", swerveSubsystem.getRawPitchDeg()) + "°");

        // Auto-recovery: sustained MT1 flip while level → reset heading from MT1
        if (consecutiveMT1FlipCount >= VisionConstants.kMT1RecoveryCycles) {
          Pose2d currentPose = swerveSubsystem.getPose();
          Pose2d recoveryPose = new Pose2d(currentPose.getTranslation(),
              Rotation2d.fromDegrees(lastMT1HeadingDeg));
          DataLogManager.log("[VISION-MT1-RECOVERY] heading " + String.format("%.1f", currentPose.getRotation().getDegrees())
              + "° → MT1 " + String.format("%.1f", lastMT1HeadingDeg)
              + "° after " + consecutiveMT1FlipCount + " cycles");
          DataLogManager.log("Vision: MT1 HEADING RECOVERY — heading "
              + String.format("%.1f", currentPose.getRotation().getDegrees())
              + " → " + String.format("%.1f", lastMT1HeadingDeg));
          swerveSubsystem.resetOdometry(recoveryPose);
          consecutiveMT1FlipCount = 0;
          mt1HeadingRecoveryCount++;
          imuSettleCycleCount = 0;
          imuSettled = false;
        }
      } else {
        consecutiveMT1FlipCount = Math.max(0, consecutiveMT1FlipCount - 1);
      }
    } else {
      consecutiveMT1FlipCount = Math.max(0, consecutiveMT1FlipCount - 1);
    }

    if (!rejectReason.isEmpty()) {
      result.rejectReason = rejectReason;
      incrementRejectCounter(rejectReason);
      publishCameraTelemetry(posePublisher, tagCountEntry, avgDistEntry, acceptedEntry, rejectEntry,
          visionPose, estimate.tagCount, estimate.avgTagDist, false, rejectReason);
      return result;
    }

    acceptCount++;
    consecutiveMT1FlipCount = 0;

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

      DataLogManager.log("[VISION-SEED] oldHdg=" + String.format("%.1f", swerveSubsystem.getHeading().getDegrees())
          + " newHdg=" + String.format("%.1f", seedPose.getRotation().getDegrees())
          + " firstFix=" + !hasEverHadFix + " tags=" + estimate.tagCount);
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

    // Publish per-camera tag IDs and latency regardless of fusion
    frontTagIdsPub.set(front.tagIds);
    backTagIdsPub.set(back.tagIds);
    // latency entries removed — development-only diagnostic

    if (!frontReady && !backReady) {
      // Nothing to fuse — clear fusion telemetry
      fusionModeEntry.setString("none");
      frontFusedStdDevEntry.setDouble(-1);
      backFusedStdDevEntry.setDouble(-1);
      visionDeltaEntry.setDouble(-1);
      return;
    }

    // Capture odometry pose BEFORE fusion for delta measurement
    Pose2d preFusionPose = swerveSubsystem.getPose();
    preFusionPosePub.set(preFusionPose);

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
        fusionModeEntry.setString("dual_agreed");
        frontFusedStdDevEntry.setDouble(stdDev);
        backFusedStdDevEntry.setDouble(stdDev);
      } else {
        // Both accepted but disagree — use normal dynamic std devs
        double frontStd = getEffectiveXYStdDev(front.baseXYStdDev);
        double backStd = getEffectiveXYStdDev(back.baseXYStdDev);
        swerveSubsystem.addVisionMeasurement(
            front.pose, front.timestamp, VecBuilder.fill(frontStd, frontStd, 9999.0));
        swerveSubsystem.addVisionMeasurement(
            back.pose, back.timestamp, VecBuilder.fill(backStd, backStd, 9999.0));
        fusionModeEntry.setString("dual_independent");
        frontFusedStdDevEntry.setDouble(frontStd);
        backFusedStdDevEntry.setDouble(backStd);
      }

      // Vision delta: average distance from odometry to both vision poses
      double frontDelta = preFusionPose.getTranslation().getDistance(front.pose.getTranslation());
      double backDelta = preFusionPose.getTranslation().getDistance(back.pose.getTranslation());
      visionDeltaEntry.setDouble((frontDelta + backDelta) / 2.0);
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

      fusionModeEntry.setString(frontReady ? "single_front" : "single_back");
      if (frontReady) {
        frontFusedStdDevEntry.setDouble(stdDev);
        backFusedStdDevEntry.setDouble(-1);
      } else {
        frontFusedStdDevEntry.setDouble(-1);
        backFusedStdDevEntry.setDouble(stdDev);
      }
      visionDeltaEntry.setDouble(
          preFusionPose.getTranslation().getDistance(single.pose.getTranslation()));
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
    double yawRate = Math.abs(swerveSubsystem.getYawRateDegPerSec());
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
    double gyroHeadingDeg = swerveSubsystem.getHeading().getDegrees();
    double headingError = Math.abs(visionHeadingDeg - gyroHeadingDeg);
    if (headingError > 180) headingError = 360 - headingError;
    lastHeadingError = headingError;
    boolean headingErrorHigh = headingError > 15.0;
    if (headingErrorHigh && !wasHeadingErrorHigh) {
      DataLogManager.log("[VISION-HEADING] error=" + String.format("%.1f", headingError)
          + "° visionHdg=" + String.format("%.1f", visionHeadingDeg)
          + " gyroHdg=" + String.format("%.1f", gyroHeadingDeg)
          + " tags=" + estimate.tagCount
          + " avgDist=" + String.format("%.2f", estimate.avgTagDist) + "m");
    }
    wasHeadingErrorHigh = headingErrorHigh;
    if (headingError > VisionConstants.kMaxHeadingErrorDeg) {
      DataLogManager.log("[VISION-REJECTED] heading_flip error=" + String.format("%.1f", headingError)
          + "° visionHdg=" + String.format("%.1f", visionHeadingDeg)
          + " gyroHdg=" + String.format("%.1f", gyroHeadingDeg)
          + " tags=" + estimate.tagCount);
      return "heading_flip";
    }

    return "";
  }

  // --- IMU mode transitions ---

  private void enterSeedMode() {
    DataLogManager.log("[VISION-IMU] -> SyncInternalImu, hdg=" + String.format("%.1f", swerveSubsystem.getHeading().getDegrees()));
    limelightFront.getSettings().withImuMode(ImuMode.SyncInternalImu).save();
    limelightBack.getSettings().withImuMode(ImuMode.SyncInternalImu).save();
    imuSettleCycleCount = 0;
    imuSettled = false;
    currentImuModeLabel = "SyncInternalImu";
    loggedDisabledSeed = false;
    DataLogManager.log("Vision: entering seed mode (SyncInternalImu)");
  }

  private void enterActiveMode() {
    DataLogManager.log("[VISION-IMU] -> ExternalImu, hdg=" + String.format("%.1f", swerveSubsystem.getHeading().getDegrees()));
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

        // MT1 heading guard — block hard reset if heading is 180° off
        boolean dualResetBlocked = false;
        if (!Double.isNaN(lastMT1HeadingDeg)) {
          double resetHdgError = Math.abs(resetPose.getRotation().getDegrees() - lastMT1HeadingDeg);
          if (resetHdgError > 180) resetHdgError = 360 - resetHdgError;
          if (resetHdgError > 120.0) {
            DataLogManager.log("[VISION-HARD-RESET-BLOCKED] dual-cam heading "
                + String.format("%.1f", resetPose.getRotation().getDegrees())
                + "° vs MT1 " + String.format("%.1f", lastMT1HeadingDeg) + "° — skipping reset");
            dualResetBlocked = true;
            hardResetCycleCount = 0;
          }
        }
        if (!dualResetBlocked) {
          DataLogManager.log("[VISION-HARD-RESET-DUAL] oldHdg=" + String.format("%.1f", swerveSubsystem.getHeading().getDegrees())
              + " newHdg=" + String.format("%.1f", resetPose.getRotation().getDegrees())
              + " drift=" + String.format("%.2f", driftMagnitude) + "m");
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

          // MT1 heading guard — block hard reset if heading is 180° off
          boolean singleResetBlocked = false;
          if (!Double.isNaN(lastMT1HeadingDeg)) {
            double resetHdgError = Math.abs(resetPose.getRotation().getDegrees() - lastMT1HeadingDeg);
            if (resetHdgError > 180) resetHdgError = 360 - resetHdgError;
            if (resetHdgError > 120.0) {
              DataLogManager.log("[VISION-HARD-RESET-BLOCKED] single-cam ("
                  + (isFrontActive ? "front" : "back") + ") heading "
                  + String.format("%.1f", resetPose.getRotation().getDegrees())
                  + "° vs MT1 " + String.format("%.1f", lastMT1HeadingDeg) + "° — skipping reset");
              singleResetBlocked = true;
              singleCameraHardResetCycleCount = 0;
            }
          }
          if (!singleResetBlocked) {
            DataLogManager.log("[VISION-HARD-RESET-SINGLE] cam=" + (isFrontActive ? "front" : "back")
                + " oldHdg=" + String.format("%.1f", swerveSubsystem.getHeading().getDegrees())
                + " newHdg=" + String.format("%.1f", resetPose.getRotation().getDegrees())
                + " drift=" + String.format("%.2f", driftMagnitude) + "m");
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

  /** Monitors camera data freshness for staleness detection. */
  private void updateCameraHealth() {
    double now = Timer.getFPGATimestamp();
    frontStale = (now - lastFrontDataTime) > VisionConstants.kCameraStaleThresholdSec;
    backStale = (now - lastBackDataTime) > VisionConstants.kCameraStaleThresholdSec;
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

  private void incrementRejectCounter(String reason) {
    switch (reason) {
      case "stale_timestamp": rejectCountStale++; break;
      case "out_of_field": rejectCountOutOfField++; break;
      case "high_yaw_rate": rejectCountYawRate++; break;
      case "too_far": rejectCountTooFar++; break;
      case "single_tag_too_far": rejectCountSingleTagFar++; break;
      case "heading_flip": rejectCountHeadingFlip++; break;
      case "mt1_flip": rejectCountMT1Flip++; break;
      default: break;
    }
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
    // pigeonRawYaw removed — redundant with odometry heading
    // detectedAlliance, firstFixStatus, imuMode, aprilTagReady — removed (logged as events)
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
    // frontStale/backStale removed from NT — only used internally for fusion logic

    // Rejection counters
    countAcceptedEntry.setDouble(acceptCount);
    countStaleEntry.setDouble(rejectCountStale);
    countOutOfFieldEntry.setDouble(rejectCountOutOfField);
    countYawRateEntry.setDouble(rejectCountYawRate);
    countTooFarEntry.setDouble(rejectCountTooFar);
    countSingleTagFarEntry.setDouble(rejectCountSingleTagFar);
    countHeadingFlipEntry.setDouble(rejectCountHeadingFlip);
    countMT1FlipEntry.setDouble(rejectCountMT1Flip);
    mt1RecoveryCountEntry.setDouble(mt1HeadingRecoveryCount);
    countMT1FlipConsecutiveEntry.setDouble(consecutiveMT1FlipCount);
    countNoDataEntry.setDouble(rejectCountNoData);

    // Field2d — overlay odometry + vision poses
    fieldOdometry.setPose(swerveSubsystem.getPose());
    if (lastFrontVisionPose != null) {
      fieldFrontVision.setPose(lastFrontVisionPose);
    }
    if (lastBackVisionPose != null) {
      fieldBackVision.setPose(lastBackVisionPose);
    }

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
