package frc.robot.simulation;

import com.revrobotics.spark.SparkSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Simulates the intake deployment arm (NEO 550 + SparkMax, 87.5:1 reduction).
 * Updates the absolute encoder position via SparkSim.
 */
public class IntakeDeploySim {

    private final SparkSim sparkSim;
    private final SingleJointedArmSim armSim;

    public IntakeDeploySim(IntakeSubsystem subsystem) {
        DCMotor motor = DCMotor.getNeo550(1);
        sparkSim = new SparkSim(subsystem.getDeployMotor(), motor);
        armSim = new SingleJointedArmSim(
            motor,
            SimConstants.kIntakeDeployGearRatio,
            SimConstants.kIntakeDeployMOI,
            SimConstants.kIntakeDeployLengthMeters,
            SimConstants.kIntakeDeployMinAngleRad,
            SimConstants.kIntakeDeployMaxAngleRad,
            true, // simulate gravity
            SimConstants.kIntakeDeployStartingAngleRad
        );
    }

    public void update() {
        double vbus = sparkSim.getBusVoltage();
        double appliedOutput = sparkSim.getAppliedOutput();
        double voltage = appliedOutput * vbus;

        armSim.setInputVoltage(voltage);
        armSim.update(SimConstants.kSimTimestepSeconds);

        // Convert arm velocity to motor RPM
        double motorVelocityRPM = Units.radiansPerSecondToRotationsPerMinute(
            armSim.getVelocityRadPerSec() * SimConstants.kIntakeDeployGearRatio
        );
        sparkSim.iterate(motorVelocityRPM, vbus, SimConstants.kSimTimestepSeconds);

        // Override absolute encoder position
        // Subsystem formula: angle_rad = 2*PI * (encoderPos / kDeployGearRatio)
        // So: encoderPos = (angle_rad / (2*PI)) * kDeployGearRatio
        double armAngleRad = armSim.getAngleRads();
        double encoderPosition = (armAngleRad / (2.0 * Math.PI)) * IntakeConstants.kDeployGearRatio;
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
