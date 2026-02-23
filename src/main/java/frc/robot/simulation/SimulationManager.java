package frc.robot.simulation;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedBattery;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

/**
 * Central orchestrator for all robot simulation.
 * Created in Robot.simulationInit(), updated in Robot.simulationPeriodic().
 * Manages the maple-sim arena, all mechanism sims, game piece tracking,
 * projectile launching, vision sim, and Mechanism2d visualization.
 */
public class SimulationManager {

    // Subsystem references
    private final SwerveSubsystem swerve;

    // Mechanism simulations
    private final ShooterSim shooterSim;
    private final HoodSim hoodSim;
    private final IntakeDeploySim intakeDeploySim;
    private final IntakeRollerSim intakeRollerSim;
    private final ConveyorSim conveyorSim;
    private final KickerSim kickerSim;
    private final ClimberSim climberSim;

    // Game piece management
    private final IntakeSimulation intakeSim;
    private final GamePieceTracker gamePieceTracker;

    // Vision simulation
    private final VisionSim visionSim;

    // Mechanism2d visualization
    private final MechanismManager mechanismManager;

    // NT publishers
    private final NetworkTable simTable;
    private final StructArrayPublisher<Pose3d> fuelPosesPublisher;
    private final StructArrayPublisher<Pose3d> projectilePosesPublisher;
    private final StructPublisher<Pose2d> groundTruthPublisher;

    public SimulationManager(RobotContainer robotContainer) {
        swerve = robotContainer.getSwerveSubsystem();

        // Initialize arena - YAGSL defaults to Arena2026Rebuilt for 2026 game
        SimulatedArena arena = SimulatedArena.getInstance();
        arena.resetFieldForAuto();

        // Create mechanism sims
        shooterSim = new ShooterSim(robotContainer.getShooterSubsystem());
        hoodSim = new HoodSim(robotContainer.getHoodSubsystem());
        intakeDeploySim = new IntakeDeploySim(robotContainer.getIntakeSubsystem());
        intakeRollerSim = new IntakeRollerSim(robotContainer.getIntakeSubsystem());
        conveyorSim = new ConveyorSim(robotContainer.getConveyorSubsystem());
        kickerSim = new KickerSim(robotContainer.getKickerSubsystem());
        climberSim = new ClimberSim(robotContainer.getClimberSubsystem());

        // Create intake simulation for game piece acquisition
        // Over-the-bumper intake on the front of the robot
        intakeSim = IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            swerve.getMapleSimDrive().get(),
            Meters.of(SimConstants.kIntakeWidthMeters),
            Meters.of(SimConstants.kIntakeExtensionMeters),
            IntakeSimulation.IntakeSide.FRONT,
            SimConstants.kIntakeCapacity
        );

        // Game piece tracking pipeline
        gamePieceTracker = new GamePieceTracker(intakeSim);

        // Vision simulation
        visionSim = new VisionSim(swerve);

        // Mechanism2d
        mechanismManager = new MechanismManager();

        // Register current draws with SimulatedBattery
        // Swerve module currents are registered automatically by YAGSL
        SimulatedBattery.addElectricalAppliances(() -> Amps.of(shooterSim.getCurrentDrawAmps()));
        SimulatedBattery.addElectricalAppliances(() -> Amps.of(hoodSim.getCurrentDrawAmps()));
        SimulatedBattery.addElectricalAppliances(() -> Amps.of(intakeDeploySim.getCurrentDrawAmps()));
        SimulatedBattery.addElectricalAppliances(() -> Amps.of(intakeRollerSim.getCurrentDrawAmps()));
        SimulatedBattery.addElectricalAppliances(() -> Amps.of(conveyorSim.getCurrentDrawAmps()));
        SimulatedBattery.addElectricalAppliances(() -> Amps.of(kickerSim.getCurrentDrawAmps()));
        SimulatedBattery.addElectricalAppliances(() -> Amps.of(climberSim.getCurrentDrawAmps()));

        // NT publishers
        simTable = NetworkTableInstance.getDefault().getTable("Simulation");
        fuelPosesPublisher = simTable
            .getStructArrayTopic("Arena/FuelPoses", Pose3d.struct).publish();
        projectilePosesPublisher = simTable
            .getStructArrayTopic("Arena/ProjectilePoses", Pose3d.struct).publish();
        groundTruthPublisher = simTable
            .getStructTopic("Robot/GroundTruthPose", Pose2d.struct).publish();
    }

    /**
     * Called every simulation periodic cycle (20ms).
     * Steps all physics sims and publishes telemetry.
     */
    public void update() {
        // Step maple-sim arena (drivetrain + field physics)
        SimulatedArena.getInstance().simulationPeriodic();

        // Step all mechanism sims
        shooterSim.update();
        hoodSim.update();
        intakeDeploySim.update();
        intakeRollerSim.update();
        conveyorSim.update();
        kickerSim.update();
        climberSim.update();

        // Control intake sim based on roller state
        if (intakeRollerSim.isRunning()) {
            intakeSim.startIntake();
        } else {
            intakeSim.stopIntake();
        }

        // Update game piece pipeline
        boolean shooterAtSpeed = shooterSim.getAngularVelocityRPM() > 500; // minimum threshold
        boolean shouldLaunch = gamePieceTracker.update(
            intakeRollerSim.isRunning(),
            conveyorSim.isRunningForward(),
            kickerSim.isRunningForward(),
            shooterAtSpeed
        );

        // Launch projectile if triggered
        if (shouldLaunch) {
            launchProjectile();
        }

        // Update vision sim
        visionSim.update();

        // Update Mechanism2d visualization
        double shooterTargetRPM = 0; // will show 0 when not commanding
        boolean fuelStaged = gamePieceTracker.isBeamBreakTripped();
        boolean fuelMoving = gamePieceTracker.getState() == GamePieceTracker.FuelState.IN_CONVEYOR
                          || gamePieceTracker.getState() == GamePieceTracker.FuelState.IN_INTAKE;

        mechanismManager.update(
            intakeDeploySim.getAngleDeg(),
            intakeRollerSim.isRunning(),
            hoodSim.getAngleDeg(),
            shooterSim.getAngularVelocityRPM(),
            shooterTargetRPM,
            climberSim.getPositionMeters(),
            fuelStaged,
            fuelMoving
        );

        // Publish NT telemetry
        publishTelemetry();
    }

    private void launchProjectile() {
        try {
            Pose2d robotPose = swerve.getMapleSimPose();
            ChassisSpeeds fieldSpeeds = swerve.getFieldVelocity();

            // Calculate exit velocity from flywheel RPM and wheel radius
            double flywheelRadPerSec = shooterSim.getAngularVelocityRPM() * 2.0 * Math.PI / 60.0;
            double exitVelocity = flywheelRadPerSec * SimConstants.kShooterWheelRadiusMeters;

            // Hood angle determines launch pitch
            double launchAngleDeg = hoodSim.getAngleDeg();

            // Create projectile
            RebuiltFuelOnFly projectile = new RebuiltFuelOnFly(
                robotPose.getTranslation(),
                new Translation2d(SimConstants.kShooterOffsetX, SimConstants.kShooterOffsetY),
                fieldSpeeds,
                robotPose.getRotation(),
                Meters.of(SimConstants.kShooterHeightMeters),
                MetersPerSecond.of(exitVelocity),
                Degrees.of(launchAngleDeg)
            );

            SimulatedArena.getInstance().addGamePieceProjectile(projectile);
        } catch (Exception e) {
            // Silently handle if maple-sim state not ready
        }
    }

    private void publishTelemetry() {
        try {
            // Game pieces on field
            List<Pose3d> fuelPoses = SimulatedArena.getInstance().getGamePiecesPosesByType("Fuel");
            fuelPosesPublisher.set(fuelPoses.toArray(new Pose3d[0]));
            simTable.getEntry("Arena/FuelCount").setInteger(fuelPoses.size());

            // In-flight projectiles (empty array published for AdvantageScope slot)
            projectilePosesPublisher.set(new Pose3d[0]);

            // Ground truth pose (debug)
            groundTruthPublisher.set(swerve.getMapleSimPose());

            // Battery telemetry
            simTable.getEntry("Battery/Voltage").setDouble(
                SimulatedBattery.getBatteryVoltage().in(edu.wpi.first.units.Units.Volts));
            simTable.getEntry("Battery/TotalCurrentDraw").setDouble(
                SimulatedBattery.getTotalCurrentDrawn().in(Amps));

            // Mechanism telemetry
            simTable.getEntry("Intake/ArmAngleDeg").setDouble(intakeDeploySim.getAngleDeg());
            simTable.getEntry("Intake/RollerRPM").setDouble(intakeRollerSim.getAngularVelocityRPM());
            simTable.getEntry("Shooter/FlywheelRPM").setDouble(shooterSim.getAngularVelocityRPM());
            simTable.getEntry("Shooter/HoodAngleDeg").setDouble(hoodSim.getAngleDeg());
            simTable.getEntry("Climber/PositionMeters").setDouble(climberSim.getPositionMeters());
        } catch (Exception e) {
            // Silently handle if state not ready
        }
    }
}
