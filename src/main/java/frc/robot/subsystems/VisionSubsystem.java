package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
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
 * Each cycle:
 * <ol>
 *   <li>Feeds the robot's heading to both Limelights (required for MegaTag2)</li>
 *   <li>Gets MegaTag2 pose estimates from both cameras</li>
 *   <li>Runs a 4-stage rejection filter (stale, field boundary, yaw rate, odometry cross-check)</li>
 *   <li>Calculates dynamic standard deviations based on tag distance and count</li>
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

  private final SwerveSubsystem swerveSubsystem;

  private final Limelight limelightFront;
  private final Limelight limelightBack;

  // MT2 for enabled-period Kalman fusion
  // Direct construction avoids YALL's singleton PoseEstimate binding (see class-level comment)
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
    Rotation3d pigeonRotation = swerveSubsystem.getGyroRotation3d();
    Rotation3d feedRotation = new Rotation3d(
        pigeonRotation.getMeasureX(),
        pigeonRotation.getMeasureY(),
        swerveSubsystem.getHeading().getMeasure());

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

    // --- Process both cameras ---
    processCamera(frontPoseEstimate, true);
    processCamera(backPoseEstimate, false);

    // --- LED feedback while disabled ---
    int totalTags = lastFrontTagCount + lastBackTagCount;
    if (!isEnabled) {
      updateDisabledLeds(totalTags);
    }

    publishSystemTelemetry();
  }

  /**
   * Core vision pipeline for a single camera.
   */
  private void processCamera(PoseEstimate poseEstimateObj, boolean isFront) {
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
      return;
    }

    PoseEstimate estimate = estimateOpt.get();
    if (isFront) lastFrontTagCount = estimate.tagCount; else lastBackTagCount = estimate.tagCount;
    Pose2d visionPose = estimate.pose.toPose2d();
    double now = Timer.getFPGATimestamp();

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
      publishCameraTelemetry(posePublisher, tagCountEntry, avgDistEntry, acceptedEntry, rejectEntry,
          visionPose, estimate.tagCount, estimate.avgTagDist, false, rejectReason);
      return;
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
      // Reuse mt1Opt fetched above for telemetry
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
    } else {
      // --- Normal enabled fusion: Kalman filter with dynamic std devs ---
      swerveSubsystem.addVisionMeasurement(
          visionPose,
          estimate.timestampSeconds,
          VecBuilder.fill(xyStdDev, xyStdDev, 9999.0));
    }

    lastAcceptTime = now;

    publishCameraTelemetry(posePublisher, tagCountEntry, avgDistEntry, acceptedEntry, rejectEntry,
        visionPose, estimate.tagCount, estimate.avgTagDist, true, "");
  }

  /**
   * 4-stage rejection filter. Returns empty string if accepted.
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

    // 4. Odometry cross-check (skip if we've never had a fix — odometry is at 0,0)
    if (hasEverHadFix) {
      double odometryJump = visionPose.getTranslation()
          .getDistance(swerveSubsystem.getPose().getTranslation());
      double odometryLimit = VisionConstants.kMaxOdometryJumpM;
      if (odometryJump > odometryLimit) {
        return "odometry_jump";
      }
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
