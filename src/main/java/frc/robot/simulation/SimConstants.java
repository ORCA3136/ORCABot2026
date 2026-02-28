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
    // 7 lbs full assembly, 17.5" pivot-to-roller, MOI = (1/3)*m*L^2 (uniform bar approx)
    public static final double kIntakeDeployMassKg = Units.lbsToKilograms(7.0); // ~3.18 kg
    public static final double kIntakeDeployLengthMeters = Units.inchesToMeters(17.5); // ~0.445 m
    public static final double kIntakeDeployMOI = (1.0 / 3.0) * kIntakeDeployMassKg
        * kIntakeDeployLengthMeters * kIntakeDeployLengthMeters; // ~0.209 kg*m^2
    public static final double kIntakeDeployGearRatio = 35.0 * 22.0 / 18.0; // 35:1 planetary + 18T->22T sprocket = 42.78:1
    // Sim angle convention: 0 = horizontal, positive = above horizontal (CCW)
    // Deployed ≈ horizontal, stowed ≈ 63° above horizontal (from encoder travel of 0.175 rot)
    // Generous hard-stop limits to avoid clamping during normal operation
    public static final double kIntakeDeployMinAngleRad = Units.degreesToRadians(-30); // below horizontal
    public static final double kIntakeDeployMaxAngleRad = Units.degreesToRadians(100); // above stowed
    public static final double kIntakeDeployStartingAngleRad = Units.degreesToRadians(63); // starts stowed

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
