// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Optional;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

/*
 * 
 * This subsystem is for all vision operations
 * Including:
 *    Limelight 
 *    Any distance or beam break sensors
 * 
 */

public class VisionSubsystem extends SubsystemBase {

  Limelight limelightOne = new Limelight(VisionConstants.limelightOneName);
  Limelight limelightTwo = new Limelight(VisionConstants.limelightTwoName);

  private LaserCan lidar;
  // private LaserCan.Measurement intakeLidar;
  // private boolean intakeStatus;

  NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  NetworkTable odometryTable = networkTable.getTable(NetworkTableNames.Odometry.kOdometry);
  NetworkTable visionTable = networkTable.getTable(NetworkTableNames.Vision.kVision);

  StructPublisher<Pose2d> visionEstimatePublisher = visionTable
      .getStructTopic(NetworkTableNames.Vision.kVisionEstimatePose2d, Pose2d.struct).publish();

  StructSubscriber<Pose2d> robotPositionSubscriber = odometryTable
      .getStructTopic(NetworkTableNames.Odometry.kRobotPose2d, Pose2d.struct).subscribe(new Pose2d());
  StructSubscriber<Rotation3d> robotRotation3dSubscriber = odometryTable
      .getStructTopic(NetworkTableNames.Odometry.kRobotRotation3d, Rotation3d.struct).subscribe(new Rotation3d());
  DoubleArraySubscriber robotAngularVelocity3dSubscriber = odometryTable
      .getDoubleArrayTopic(NetworkTableNames.Odometry.kRobotAngularVelocity3d).subscribe(new double[] {});

  public VisionSubsystem() {
    // lidar = new LaserCan(Constants.CanIdConstants.kLidarCanId);

    limelightOne.getSettings()
         .withLimelightLEDMode(LEDMode.PipelineControl)
         .withCameraOffset(Pose3d.kZero)
         .withImuMode(ImuMode.SyncInternalImu)
         .save();
    limelightTwo.getSettings()
         .withLimelightLEDMode(LEDMode.PipelineControl)
         .withCameraOffset(Pose3d.kZero)
         .withImuMode(ImuMode.SyncInternalImu)
         .save();
  }

  /**
   * @return The distance before the lidar sees something
   */
  public double getLidarMeasurement() {
    Measurement it = lidar.getMeasurement();

    /*
    MOVE TO CONSTANTS
    NetworkTableInstance.getDefault().getTable("Lidar").getEntry("Lidar status").setDouble(it.status);
    */
    
    if (it == null) 
      return 999999;
    return it.distance_mm;
  }

  /**
   * @return True if limelightOne sees a valid tag
   */
  public boolean getLeftTV() {
    return false; // limelightOne.getData().get;
  }
  /**
   * @return True if limelightTwo sees a valid tag
   */
  public boolean getRightTV() {
    return false;
  }
  /**
   * @return True if any limelight sees a valid tag
   */
  public boolean getTV() {
    return getLeftTV() || getRightTV();
  }

  


  /**
   * 
   */
  public void updateRobotPosition() {
    // Get current robot position
    Pose2d currentPose = robotPositionSubscriber.get();

    Limelight[] limelights = {limelightOne, limelightTwo};
    String[] limelightNames = {VisionConstants.limelightOneName, VisionConstants.limelightTwoName};

    boolean updatePose = false;
    int bestLimelight = -1;
    double maxFitness = 1000;

    // Check for each limelight
    for (int i = 0; i <= 1; i++) {
      // Get and check if there is a valid estimate
      Optional<PoseEstimate> visionEstimate = limelights[i]
          .createPoseEstimator(EstimationMode.MEGATAG2).getPoseEstimate();
      if (visionEstimate.isEmpty()) continue;

      double[] stddevs = NetworkTableInstance.getDefault().getTable(limelightNames[i])
          .getEntry("stddevs").getDoubleArray(new double[12]);
      
      // If the estimate is more accurate than the current pose then update
      if (MathUtil.isNear(visionEstimate.get().pose.getX(), currentPose.getX(), stddevs[6]) && 
          MathUtil.isNear(visionEstimate.get().pose.getY(), currentPose.getY(), stddevs[7]) && 
          MathUtil.isNear(visionEstimate.get().pose.getRotation().getZ(), currentPose.getRotation().getRadians(), stddevs[11]))
      {
        updatePose = true;
        continue;
      }

      // Calculate fitness score
      double timeFitness = 0; // time since last update
      double positionFitness = 0; // stddevs
      double velocityFitness = 0; // How fast we are driving/accelerating    // Odometry (jostling)
      double zFitness = 0; // If the robot isn't on the ground
      double userFitness = 0; // Decrease fitness at user request

      double totalFitness = timeFitness + positionFitness + velocityFitness + zFitness - userFitness;
      maxFitness = (totalFitness < maxFitness ? totalFitness : maxFitness);
    }

    if (updatePose && bestLimelight != -1) {
      PoseEstimate estimate = limelights[bestLimelight].createPoseEstimator(EstimationMode.MEGATAG2).getPoseEstimate().get();
      visionEstimatePublisher.set(estimate.pose.toPose2d());
      visionTable.getEntry(NetworkTableNames.Vision.kVisionEstimateTimestamp).setDouble(estimate.timestampSeconds);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    limelightOne.getSettings()
		 .withRobotOrientation(new Orientation3d(robotRotation3dSubscriber.get(),
				new AngularVelocity3d(DegreesPerSecond.of(robotAngularVelocity3dSubscriber.get()[0]),
															DegreesPerSecond.of(robotAngularVelocity3dSubscriber.get()[0]),
															DegreesPerSecond.of(robotAngularVelocity3dSubscriber.get()[0]))))
		 .save();
    limelightTwo.getSettings()
		 .withRobotOrientation(new Orientation3d(robotRotation3dSubscriber.get(),
				new AngularVelocity3d(DegreesPerSecond.of(robotAngularVelocity3dSubscriber.get()[0]),
															DegreesPerSecond.of(robotAngularVelocity3dSubscriber.get()[0]),
															DegreesPerSecond.of(robotAngularVelocity3dSubscriber.get()[0]))))
		 .save();

    // NetworkTableInstance.getDefault().getTable("Lidar").getEntry("Lidar distance").setDouble(getLidarMeasurement());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Command getLLSeedCommand() {
    return Commands.runOnce(() -> {
      limelightOne.getSettings().withImuMode(ImuMode.SyncInternalImu).save(); 
      limelightTwo.getSettings().withImuMode(ImuMode.SyncInternalImu).save();}, 
      (Subsystem[]) null);
  }

  public Command getLLInternalCommand() {
    return Commands.runOnce(() -> {
      limelightOne.getSettings().withImuMode(ImuMode.InternalImu).save(); 
      limelightTwo.getSettings().withImuMode(ImuMode.InternalImu).save();}, 
      (Subsystem[]) null);
}
}