package frc.robot.simulation;

import java.util.Random;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.NetworkTableNames;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Simulates Limelight vision measurements by:
 * 1. Getting ground truth pose from maple-sim drivetrain
 * 2. Adding Gaussian noise scaled by distance
 * 3. Rate-limiting to ~15 Hz with simulated latency
 * 4. Publishing to the same NT keys that SwerveSubsystem.getVisionUpdate() reads
 */
public class VisionSim {

    private final SwerveSubsystem swerve;
    private final Random random = new Random();

    private double lastUpdateTime = 0;
    private final double updatePeriod = 1.0 / SimConstants.kVisionUpdateRateHz;

    private final NetworkTable visionTable;
    private final StructPublisher<Pose2d> visionPosePublisher;

    public VisionSim(SwerveSubsystem swerve) {
        this.swerve = swerve;
        visionTable = NetworkTableInstance.getDefault().getTable(NetworkTableNames.Vision.kTable);
        visionPosePublisher = visionTable
            .getStructTopic(NetworkTableNames.Vision.kVisionEstimatePose2d, Pose2d.struct).publish();
    }

    public void update() {
        double now = Timer.getFPGATimestamp();

        // Rate-limit to simulated Limelight update rate
        if (now - lastUpdateTime < updatePeriod) {
            return;
        }
        lastUpdateTime = now;

        // Get ground truth pose from maple-sim
        Pose2d groundTruth;
        try {
            groundTruth = swerve.getMapleSimPose();
        } catch (Exception e) {
            return; // maple-sim drive not available
        }

        // Add Gaussian noise
        double noisyX = groundTruth.getX()
            + random.nextGaussian() * SimConstants.kVisionPositionStdDev;
        double noisyY = groundTruth.getY()
            + random.nextGaussian() * SimConstants.kVisionPositionStdDev;
        double noisyRot = groundTruth.getRotation().getRadians()
            + random.nextGaussian() * SimConstants.kVisionRotationStdDev;

        Pose2d noisyPose = new Pose2d(noisyX, noisyY, new Rotation2d(noisyRot));

        // Publish with simulated latency
        double timestamp = now - SimConstants.kVisionLatencySeconds;

        visionPosePublisher.set(noisyPose);
        visionTable.getEntry(NetworkTableNames.Vision.kVisionEstimateTimestamp).setDouble(timestamp);
    }
}
