package frc.robot.simulation;

import edu.wpi.first.math.util.Units;

/**
 * Physical parameters for simulation.
 * All MOI values are estimates - tune by comparing sim spin-up time to real robot.
 */
public final class SimConstants {

    // Shooter flywheel
    public static final double kShooterMOI = 0.005; // kg*m^2 TODO: measure from CAD
    public static final double kShooterGearRatio = 1.0; // direct drive
    public static final double kShooterWheelRadiusMeters = Units.inchesToMeters(2.0); // TODO: measure

    // Hood (SingleJointedArm)
    public static final double kHoodMOI = 0.01; // kg*m^2 TODO: measure from CAD
    public static final double kHoodLengthMeters = 0.15; // m TODO: measure
    public static final double kHoodMassKg = 0.5; // kg TODO: measure
    public static final double kHoodGearRatio = 12.0; // motor gear ratio
    public static final double kHoodMinAngleRad = 0.0;
    public static final double kHoodMaxAngleRad = Units.degreesToRadians(45);
    public static final double kHoodStartingAngleRad = 0.0;

    // Intake deploy (SingleJointedArm)
    public static final double kIntakeDeployMOI = 0.02; // kg*m^2 TODO: measure from CAD
    public static final double kIntakeDeployLengthMeters = 0.2; // m TODO: measure
    public static final double kIntakeDeployMassKg = 1.0; // kg TODO: measure
    public static final double kIntakeDeployGearRatio = 87.5; // total reduction
    public static final double kIntakeDeployMinAngleRad = 0.0;
    public static final double kIntakeDeployMaxAngleRad = Units.degreesToRadians(180); // 0.5 rot
    public static final double kIntakeDeployStartingAngleRad = 0.0;

    // Intake roller
    public static final double kIntakeRollerMOI = 0.003; // kg*m^2 TODO: measure from CAD
    public static final double kIntakeRollerGearRatio = 1.0;

    // Conveyor
    public static final double kConveyorMOI = 0.003; // kg*m^2 TODO: measure from CAD
    public static final double kConveyorGearRatio = 1.0;

    // Kicker
    public static final double kKickerMOI = 0.002; // kg*m^2 TODO: measure from CAD
    public static final double kKickerGearRatio = 1.0;

    // Climber (ElevatorSim)
    public static final double kClimberGearRatio = (28.0 / 11.0) * 125.0; // 318.18:1
    public static final double kClimberDrumRadiusMeters = 0.02; // m TODO: measure
    public static final double kClimberCarriageMassKg = 50.0; // robot weight when climbing
    public static final double kClimberMinHeightMeters = 0.0;
    public static final double kClimberMaxHeightMeters = 0.5;

    // Intake simulation
    public static final double kIntakeWidthMeters = 0.4; // width of intake zone TODO: measure
    public static final double kIntakeExtensionMeters = 0.15; // how far intake extends past bumpers TODO: measure
    public static final int kIntakeCapacity = 5; // max fuel held

    // Fuel game piece (2026 REBUILT)
    public static final double kFuelDiameterMeters = Units.inchesToMeters(5.91);
    public static final double kFuelMassKg = Units.lbsToKilograms(0.5);

    // Shooter geometry for projectile launching
    public static final double kShooterHeightMeters = 0.4; // height of shooter exit TODO: measure
    public static final double kShooterOffsetX = 0.0; // shooter X offset from robot center TODO: measure
    public static final double kShooterOffsetY = 0.0; // shooter Y offset from robot center TODO: measure

    // Vision simulation
    public static final double kVisionPositionStdDev = 0.05; // meters
    public static final double kVisionRotationStdDev = Units.degreesToRadians(2.0); // radians
    public static final double kVisionUpdateRateHz = 15.0;
    public static final double kVisionLatencySeconds = 0.04; // 40ms

    // Simulation timestep
    public static final double kSimTimestepSeconds = 0.020;

    // Beam break DIO port (prepare for physical wiring)
    public static final int kBeamBreakDIOPort = 0; // TODO: assign when wired

    private SimConstants() {}
}
