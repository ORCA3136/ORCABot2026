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
    private final IntakeDeploySim intakeDeploySim;
    private final IntakeRollerSim intakeRollerSim;
    private final ConveyorSim conveyorSim;
    private final KickerSim kickerSim;

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
        intakeDeploySim = new IntakeDeploySim(robotContainer.getIntakeSubsystem());
        intakeRollerSim = new IntakeRollerSim(robotContainer.getIntakeSubsystem());
        conveyorSim = new ConveyorSim(conveyorSubsystem);
        kickerSim = new KickerSim(kickerSubsystem);

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
        intakeDeploySim.update();
        intakeRollerSim.update();
        conveyorSim.update();
        kickerSim.update();

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

        // Commented out because I couldn'r fix the launch fuel method in ArenaSimulator
        // if (launched) {
        //     arenaSimulation.launchFuel(
        //         shooterSim.getAngularVelocityRPM()
        //     );
        // }

        // 7. Update vision simulation
        visionSim.update();

        // 8. Update Mechanism2d visualization
        // Deuce: rack & pinion — convert motor rotations to a pseudo-angle for Mechanism2d
        // (0 motor rot = 0°, max extension ≈ 90° for visualization)
        double intakeDeployAngleDeg = (intakeDeploySim.getPositionMotorRotations()
            / frc.robot.Constants.IntakeConstants.kMaxExtension) * 90.0;
        mechanismManager.update(
            intakeDeployAngleDeg,
            intakeRollerSim.isRunning(),
            shooterSim.getAngularVelocityRPM(),
            shooterSubsystem.getShooterTarget(),
            gamePieceTracker.isBeamBreakTripped(),
            gamePieceTracker.getState() == GamePieceTracker.FuelState.IN_CONVEYOR
                || gamePieceTracker.getState() == GamePieceTracker.FuelState.IN_INTAKE
        );

        // 9. Publish telemetry
        publishTelemetry();
    }

    private void publishTelemetry() {
        try {
            simTable.getEntry("Intake/PositionMotorRot").setDouble(intakeDeploySim.getPositionMotorRotations());
            simTable.getEntry("Intake/RollerRPM").setDouble(intakeRollerSim.getAngularVelocityRPM());
            simTable.getEntry("Shooter/FlywheelRPM").setDouble(shooterSim.getAngularVelocityRPM());

            // Publish field fuel positions for AdvantageScope 3D visualization
            fieldFuelPublisher.set(arenaSimulation.getFieldFuelPoses());
        } catch (Exception e) {
            // Silently handle if state not ready
        }
    }
}
