package frc.robot.simulation;

import com.revrobotics.spark.SparkSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.ConveyorSubsystem;

/**
 * Simulates the conveyor belt (Vortex, duty cycle control).
 */
public class ConveyorSim {

    private final SparkSim sparkSim;
    private final FlywheelSim flywheelSim;

    public ConveyorSim(ConveyorSubsystem subsystem) {
        DCMotor motor = DCMotor.getNeoVortex(1);
        sparkSim = new SparkSim(subsystem.getMotor(), motor);
        flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(motor, SimConstants.kConveyorMOI, SimConstants.kConveyorGearRatio),
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

    /** @return true if conveyor is running forward */
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
