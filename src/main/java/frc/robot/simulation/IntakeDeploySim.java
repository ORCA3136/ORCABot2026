package frc.robot.simulation;

import com.revrobotics.spark.SparkSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Simulates the intake deployment rack & pinion (Vortex + SparkFlex, ~30.56:1 reduction).
 * Updates the relative encoder position via SparkSim.
 *
 * Since WPILib doesn't have a built-in linear mechanism sim that maps well to a
 * rack & pinion, we use a simple velocity integration model.
 */
public class IntakeDeploySim {

    private final SparkSim sparkSim;
    private double positionMotorRotations = 0; // Motor rotations from home

    public IntakeDeploySim(IntakeSubsystem subsystem) {
        DCMotor motor = DCMotor.getNeoVortex(1);
        sparkSim = new SparkSim(subsystem.getDeployMotor(), motor);
    }

    public void update() {
        double vbus = sparkSim.getBusVoltage();
        double appliedOutput = sparkSim.getAppliedOutput();

        // Simple model: compute motor velocity from applied voltage
        DCMotor motor = DCMotor.getNeoVortex(1);
        double voltage = appliedOutput * vbus;
        // Approximate: free speed proportional to voltage, no load
        double freeSpeedRadPerSec = motor.freeSpeedRadPerSec * (voltage / 12.0);
        double motorRPM = Units.radiansPerSecondToRotationsPerMinute(freeSpeedRadPerSec);

        // Integrate position
        double dt = SimConstants.kSimTimestepSeconds;
        positionMotorRotations += (motorRPM / 60.0) * dt;

        // Clamp to physical limits
        if (positionMotorRotations < 0) {
            positionMotorRotations = 0;
            motorRPM = 0;
        } else if (positionMotorRotations > IntakeConstants.kMaxExtension) {
            positionMotorRotations = IntakeConstants.kMaxExtension;
            motorRPM = 0;
        }

        sparkSim.iterate(motorRPM, vbus, dt);
    }

    public double getCurrentDrawAmps() {
        // Simplified — return a small value for now
        return 2.0;
    }

    public double getPositionMotorRotations() {
        return positionMotorRotations;
    }
}
