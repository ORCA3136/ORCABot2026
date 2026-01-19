// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightResults;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
// import frc.robot.Robot;
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

  Limelight limelightOne = new Limelight(Constants.VisionConstants.limelightOneName);
  Limelight limelightTwo = new Limelight(Constants.VisionConstants.limelightTwoName);

  private LaserCan lidar;
  private LaserCan.Measurement intakeLidar;
  private boolean intakeStatus;

  NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  NetworkTable odometryTable = networkTable.getTable(Constants.NetworkTableNames.Odometry.kOdometry);

  StructSubscriber<Pose2d> robotPositionSubscriber = networkTable
      .getStructTopic(Constants.NetworkTableNames.Odometry.kRobotPose2d, Pose2d.struct).subscribe(new Pose2d());

  StructSubscriber<Rotation3d> robotRotation3dSubscriber = odometryTable
      .getStructTopic(Constants.NetworkTableNames.Odometry.kRobotRotation3d, Rotation3d.struct).subscribe(new Rotation3d());
  DoubleArraySubscriber robotAngularVelocity3dSubscriber = odometryTable
      .getDoubleArrayTopic(Constants.NetworkTableNames.Odometry.kRobotAngularVelocity3d).subscribe(new double[] {});

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
      robotPositionSubscriber.get();
    // Get limelight position estimate
      Optional<PoseEstimate> visionEstimateOne = limelightOne
        .createPoseEstimator(EstimationMode.MEGATAG2).getPoseEstimate();
      Optional<PoseEstimate> visionEstimateTwo = limelightTwo
        .createPoseEstimator(EstimationMode.MEGATAG2).getPoseEstimate();
    // Get limelight STD DEVs X, Y, Yaw
      Optional<LimelightResults> resultsOne = limelightOne.getLatestResults();
      if (resultsOne.isEmpty())
      resultsOne.get().

    // Check for each limelight
      // If no valid target -- continue

      // If current position is outside of STD DEVs update position

      // If STD DEVs are less than default value scaled update position
        // Scaling includes:
          // User influence
          // Time since last update
          // Odometry (jostling)

      // 



      // robot_orientation_set
      // stddevs
      // botpose_orb_wpiblue
      // imumode_set -- 1 use external and calibrate internal -- 2 use internal

      // setVisionMeasurementStdDevs;
      // addVisionMeasurement
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    limelight.getSettings()
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
}