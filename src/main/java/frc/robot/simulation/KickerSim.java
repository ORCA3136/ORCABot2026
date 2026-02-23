package frc.robot.simulation;

import com.revrobotics.spark.SparkSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.KickerSubsystem;

/**
 * Simulates the kicker wheel (Vortex, duty cycle control).
 */
public class KickerSim {

    private final SparkSim sparkSim;
    private final FlywheelSim flywheelSim;

    public KickerSim(KickerSubsystem subsystem) {
        DCMotor motor = DCMotor.getNeoVortex(1);
        sparkSim = new SparkSim(subsystem.getMotor(), motor);
        flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(motor, SimConstants.kKickerMOI, SimConstants.kKickerGearRatio),
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

    /** @return true if kicker is running forward */
    public boolean isRunningForward() {
        return flywheelSim.getAngularVelocityRPM() > 100;
    }

    public double getCurrentDrawAmps() {
        return flywheelSim.getCurrentDrawAmps();
    }

    public double getAngularVelocityRPM() {
        return flywheelSim.getAngularVelocityRPM();
    }
}
