package frc.robot.simulation;

import com.revrobotics.spark.SparkSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Simulates the intake roller (Vortex, duty cycle control).
 */
public class IntakeRollerSim {

    private final SparkSim sparkSim;
    private final FlywheelSim flywheelSim;

    public IntakeRollerSim(IntakeSubsystem subsystem) {
        DCMotor motor = DCMotor.getNeoVortex(1);
        sparkSim = new SparkSim(subsystem.getIntakeMotor(), motor);
        flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(motor, SimConstants.kIntakeRollerMOI, SimConstants.kIntakeRollerGearRatio),
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

    /** @return true if intake roller is spinning (above threshold) */
    public boolean isRunning() {
        return Math.abs(flywheelSim.getAngularVelocityRPM()) > 100;
    }

    public double getCurrentDrawAmps() {
        return flywheelSim.getCurrentDrawAmps();
    }

    public double getAngularVelocityRPM() {
        return flywheelSim.getAngularVelocityRPM();
    }
}
