package frc.robot.simulation;

import com.revrobotics.spark.SparkSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.ClimberSubsystem;

/**
 * Simulates the dual-Vortex climber as an elevator (rope spool).
 * Secondary motor follows primary, so we only simulate the primary.
 * Uses relative encoder (no absolute encoder override needed).
 */
public class ClimberSim {

    private final SparkSim sparkSim;
    private final ElevatorSim elevatorSim;

    public ClimberSim(ClimberSubsystem subsystem) {
        DCMotor motor = DCMotor.getNeoVortex(2);
        sparkSim = new SparkSim(subsystem.getPrimaryMotor(), motor);
        elevatorSim = new ElevatorSim(
            motor,
            SimConstants.kClimberGearRatio,
            SimConstants.kClimberCarriageMassKg,
            SimConstants.kClimberDrumRadiusMeters,
            SimConstants.kClimberMinHeightMeters,
            SimConstants.kClimberMaxHeightMeters,
            true, // simulate gravity
            0.0 // starting height
        );
    }

    public void update() {
        double vbus = sparkSim.getBusVoltage();
        double appliedOutput = sparkSim.getAppliedOutput();
        double voltage = appliedOutput * vbus;

        elevatorSim.setInputVoltage(voltage);
        elevatorSim.update(SimConstants.kSimTimestepSeconds);

        // Convert linear velocity to motor RPM
        // linear velocity (m/s) = motor angular velocity (rad/s) * drum radius / gear ratio
        // motor RPM = linear velocity / drum radius * gear ratio * 60 / (2*PI)
        double linearVelocity = elevatorSim.getVelocityMetersPerSecond();
        double motorVelocityRPM = Units.radiansPerSecondToRotationsPerMinute(
            linearVelocity / SimConstants.kClimberDrumRadiusMeters * SimConstants.kClimberGearRatio
        );
        sparkSim.iterate(motorVelocityRPM, vbus, SimConstants.kSimTimestepSeconds);
    }

    public double getCurrentDrawAmps() {
        return elevatorSim.getCurrentDrawAmps();
    }

    public double getPositionMeters() {
        return elevatorSim.getPositionMeters();
    }

    public double getVelocityMetersPerSecond() {
        return elevatorSim.getVelocityMetersPerSecond();
    }
}
