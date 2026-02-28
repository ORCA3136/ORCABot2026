package frc.robot.simulation;

import com.revrobotics.spark.SparkSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Simulates the dual-NEO-550 hood mechanism as a single-jointed arm.
 * Updates the absolute encoder position via SparkSim so the subsystem
 * PID reads correct simulated values.
 */
public class HoodSim {

    private final SparkSim sparkSim;
    private final SingleJointedArmSim armSim;

    public HoodSim(ShooterSubsystem subsystem) {
        DCMotor motor = DCMotor.getNeo550(2);
        sparkSim = new SparkSim(subsystem.getPrimaryHoodMotor(), motor);
        armSim = new SingleJointedArmSim(
            motor,
            SimConstants.kHoodGearRatio,
            SimConstants.kHoodMOI,
            SimConstants.kHoodLengthMeters,
            SimConstants.kHoodMinAngleRad,
            SimConstants.kHoodMaxAngleRad,
            true, // simulate gravity
            SimConstants.kHoodStartingAngleRad
        );
    }

    public void update() {
        double vbus = sparkSim.getBusVoltage();
        double appliedOutput = sparkSim.getAppliedOutput();
        double voltage = appliedOutput * vbus;

        armSim.setInputVoltage(voltage);
        armSim.update(SimConstants.kSimTimestepSeconds);

        // Convert arm angle (rad) to motor velocity (RPM) for SparkSim.iterate()
        double motorVelocityRPM = Units.radiansPerSecondToRotationsPerMinute(
            armSim.getVelocityRadPerSec() * SimConstants.kHoodGearRatio
        );
        sparkSim.iterate(motorVelocityRPM, vbus, SimConstants.kSimTimestepSeconds);

        // Override absolute encoder position to match simulated arm angle
        // The hood encoder reads in motor rotations with the positionConversionFactor applied
        // Subsystem formula: angle_rad = 2*PI * ((encoderPos - offset) / (motorGearRatio * encoderGearRatio))
        // So: encoderPos = offset + (angle_rad / (2*PI)) * motorGearRatio * encoderGearRatio
        double armAngleRad = armSim.getAngleRads();
        double encoderPosition = HoodConstants.kEncoderOffset
            + (armAngleRad / (2.0 * Math.PI)) * HoodConstants.kMotorGearRatio * HoodConstants.kEncoderGearRatio;

        // The absolute encoder has positionConversionFactor of kMotorGearRatio applied in config,
        // so we set the raw position accounting for that
        sparkSim.getAbsoluteEncoderSim().setPosition(encoderPosition);
    }

    public double getCurrentDrawAmps() {
        return armSim.getCurrentDrawAmps();
    }

    public double getAngleRad() {
        return armSim.getAngleRads();
    }

    public double getAngleDeg() {
        return Units.radiansToDegrees(armSim.getAngleRads());
    }
}
