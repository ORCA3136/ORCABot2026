package frc.robot.simulation;

import java.util.Random;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Simulates Limelight vision measurements by:
 * 1. Getting the current odometry pose as ground truth
 * 2. Adding Gaussian noise scaled by distance
 * 3. Rate-limiting to ~15 Hz with simulated latency
 * 4. Fusing directly into the swerve drive's pose estimator
 */
public class VisionSim {

    private final SwerveSubsystem swerve;
    private final Random random = new Random();

    private double lastUpdateTime = 0;
    private final double updatePeriod = 1.0 / SimConstants.kVisionUpdateRateHz;

    public VisionSim(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    public void update() {
        double now = Timer.getFPGATimestamp();

        // Rate-limit to simulated Limelight update rate
        if (now - lastUpdateTime < updatePeriod) {
            return;
        }
        lastUpdateTime = now;

        // Get odometry pose as ground truth for vision noise simulation
        Pose2d groundTruth = swerve.getPose();

        // Guard against corrupted sim state (NaN pose from MapleSim motor model mismatch)
        if (Double.isNaN(groundTruth.getX()) || Double.isNaN(groundTruth.getY())
            || Double.isNaN(groundTruth.getRotation().getCos())) {
            return;
        }

        // Add Gaussian noise
        double noisyX = groundTruth.getX()
            + random.nextGaussian() * SimConstants.kVisionPositionStdDev;
        double noisyY = groundTruth.getY()
            + random.nextGaussian() * SimConstants.kVisionPositionStdDev;
        double noisyRot = groundTruth.getRotation().getRadians()
            + random.nextGaussian() * SimConstants.kVisionRotationStdDev;

        Pose2d noisyPose = new Pose2d(noisyX, noisyY, new Rotation2d(noisyRot));

        // Fuse directly into the pose estimator with simulated latency and std devs
        double timestamp = now - SimConstants.kVisionLatencySeconds;
        swerve.addVisionMeasurement(noisyPose, timestamp,
            VecBuilder.fill(
                SimConstants.kVisionPositionStdDev,
                SimConstants.kVisionPositionStdDev,
                SimConstants.kVisionRotationStdDev));
    }
}
