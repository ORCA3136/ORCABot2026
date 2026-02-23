package frc.robot.simulation;

import com.revrobotics.spark.SparkSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Simulates the dual-Vortex shooter flywheel.
 * Reads motor applied output via SparkSim, feeds voltage into FlywheelSim,
 * then writes back simulated velocity via SparkSim.iterate().
 */
public class ShooterSim {

    private final SparkSim sparkSim;
    private final FlywheelSim flywheelSim;

    public ShooterSim(ShooterSubsystem subsystem) {
        DCMotor motor = DCMotor.getNeoVortex(2);
        sparkSim = new SparkSim(subsystem.getPrimaryMotor(), motor);
        flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(motor, SimConstants.kShooterMOI, SimConstants.kShooterGearRatio),
            motor
        );
    }

    public void update() {
        double vbus = sparkSim.getBusVoltage();
        double appliedOutput = sparkSim.getAppliedOutput();
        double voltage = appliedOutput * vbus;

        flywheelSim.setInputVoltage(voltage);
        flywheelSim.update(SimConstants.kSimTimestepSeconds);

        double rpm = flywheelSim.getAngularVelocityRPM();
        sparkSim.iterate(rpm, vbus, SimConstants.kSimTimestepSeconds);
    }

    public double getCurrentDrawAmps() {
        return flywheelSim.getCurrentDrawAmps();
    }

    public double getAngularVelocityRPM() {
        return flywheelSim.getAngularVelocityRPM();
    }
}
