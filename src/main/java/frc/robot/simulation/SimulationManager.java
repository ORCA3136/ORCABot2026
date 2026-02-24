package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Central orchestrator for all robot simulation.
 * Created in Robot.simulationInit(), updated in Robot.simulationPeriodic().
 *
 * Uses WPILib physics sims (FlywheelSim, SingleJointedArmSim, ElevatorSim)
 * for mechanism simulation, maple-sim for arena/game piece physics,
 * plus vision and game piece tracking.
 */
public class SimulationManager {

    // Mechanism simulations
    private final ShooterSim shooterSim;
    private final HoodSim hoodSim;
    private final IntakeDeploySim intakeDeploySim;
    private final IntakeRollerSim intakeRollerSim;
    private final ConveyorSim conveyorSim;
    private final KickerSim kickerSim;
    private final ClimberSim climberSim;

    // Arena simulation (maple-sim bridge)
    private final ArenaSimulation arenaSimulation;

    // Vision simulation
    private final VisionSim visionSim;

    // Game piece tracking
    private final GamePieceTracker gamePieceTracker;

    // Mechanism2d visualization
    private final MechanismManager mechanismManager;

    // Subsystem references for querying motor state
    private final ShooterSubsystem shooterSubsystem;
    private final ConveyorSubsystem conveyorSubsystem;
    private final KickerSubsystem kickerSubsystem;

    // NT publishers
    private final NetworkTable simTable;
    private final StructArrayPublisher<Pose3d> fieldFuelPublisher;

    public SimulationManager(RobotContainer robotContainer) {
        // Store subsystem references for motor state queries
        shooterSubsystem = robotContainer.getShooterSubsystem();
        conveyorSubsystem = robotContainer.getConveyorSubsystem();
        kickerSubsystem = robotContainer.getKickerSubsystem();

        // Create mechanism sims
        shooterSim = new ShooterSim(shooterSubsystem);
        hoodSim = new HoodSim(robotContainer.getHoodSubsystem());
        intakeDeploySim = new IntakeDeploySim(robotContainer.getIntakeSubsystem());
        intakeRollerSim = new IntakeRollerSim(robotContainer.getIntakeSubsystem());
        conveyorSim = new ConveyorSim(conveyorSubsystem);
        kickerSim = new KickerSim(kickerSubsystem);
        climberSim = new ClimberSim(robotContainer.getClimberSubsystem());

        // Arena simulation (maple-sim game piece physics)
        arenaSimulation = new ArenaSimulation(robotContainer.getSwerveSubsystem());

        // Vision simulation (publishes noisy pose estimates to NT)
        visionSim = new VisionSim(robotContainer.getSwerveSubsystem());

        // Game piece tracking state machine
        gamePieceTracker = new GamePieceTracker();

        // Mechanism2d
        mechanismManager = new MechanismManager();

        // NT publishers
        simTable = NetworkTableInstance.getDefault().getTable("Simulation");
        fieldFuelPublisher = simTable
            .getStructArrayTopic("FieldFuel", Pose3d.struct).publish();
    }

    /**
     * Called every simulation periodic cycle (20ms).
     * Steps all mechanism physics, arena physics, game piece tracking, and publishes telemetry.
     */
    public void update() {
        // 1. Step all mechanism sims
        shooterSim.update();
        hoodSim.update();
        intakeDeploySim.update();
        intakeRollerSim.update();
        conveyorSim.update();
        kickerSim.update();
        climberSim.update();

        // 2. Step arena physics (maple-sim)
        arenaSimulation.update();

        // 3. Signal intake state to arena
        boolean intakeRunning = intakeRollerSim.isRunning();
        arenaSimulation.setIntakeRunning(intakeRunning);

        // 4. Check for fuel acquired from the field
        int newFuel = arenaSimulation.consumeObtainedFuel();
        for (int i = 0; i < newFuel; i++) {
            gamePieceTracker.acquireFuel();
        }

        // 5. Update game piece tracking state machine
        boolean conveyorForward = conveyorSubsystem.getConveyorVelocity() > 10;
        boolean kickerForward = kickerSubsystem.getKickerVelocity() > 10;
        boolean shooterAtSpeed = shooterSubsystem.getShooterTarget() > 0
            && Math.abs(shooterSim.getAngularVelocityRPM() - shooterSubsystem.getShooterTarget())
                < shooterSubsystem.getShooterTarget() * 0.05;

        boolean launched = gamePieceTracker.update(
            intakeRunning, conveyorForward, kickerForward, shooterAtSpeed);

        // 6. If fuel was launched, create projectile in arena
        if (launched) {
            arenaSimulation.launchFuel(
                shooterSim.getAngularVelocityRPM(),
                hoodSim.getAngleDeg()
            );
        }

        // 7. Update vision simulation
        visionSim.update();

        // 8. Update Mechanism2d visualization
        mechanismManager.update(
            intakeDeploySim.getAngleDeg(),
            intakeRollerSim.isRunning(),
            hoodSim.getAngleDeg(),
            shooterSim.getAngularVelocityRPM(),
            shooterSubsystem.getShooterTarget(),
            climberSim.getPositionMeters(),
            gamePieceTracker.isBeamBreakTripped(),
            gamePieceTracker.getState() == GamePieceTracker.FuelState.IN_CONVEYOR
                || gamePieceTracker.getState() == GamePieceTracker.FuelState.IN_INTAKE
        );

        // 9. Publish telemetry
        publishTelemetry();
    }

    private void publishTelemetry() {
        try {
            simTable.getEntry("Intake/ArmAngleDeg").setDouble(intakeDeploySim.getAngleDeg());
            simTable.getEntry("Intake/RollerRPM").setDouble(intakeRollerSim.getAngularVelocityRPM());
            simTable.getEntry("Shooter/FlywheelRPM").setDouble(shooterSim.getAngularVelocityRPM());
            simTable.getEntry("Shooter/HoodAngleDeg").setDouble(hoodSim.getAngleDeg());
            simTable.getEntry("Climber/PositionMeters").setDouble(climberSim.getPositionMeters());

            // Publish field fuel positions for AdvantageScope 3D visualization
            fieldFuelPublisher.set(arenaSimulation.getFieldFuelPoses());
        } catch (Exception e) {
            // Silently handle if state not ready
        }
    }
}
