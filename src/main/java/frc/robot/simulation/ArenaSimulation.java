package frc.robot.simulation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation.IntakeSide;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

/**
 * Bridges maple-sim's SimulatedArena with our simulation framework.
 * Handles intake game piece detection, projectile launching, and field fuel telemetry.
 *
 * The Arena2026Rebuilt instance must be set via SimulatedArena.overrideInstance()
 * BEFORE this class is constructed (done in Robot constructor).
 */
public class ArenaSimulation {

    private final IntakeSimulation intakeSim;
    private final SwerveSubsystem swerveSubsystem;

    public ArenaSimulation(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        // Get maple-sim drive simulation from YAGSL
        var driveSim = swerveSubsystem.getSwerveDrive()
            .getMapleSimDrive()
            .orElseThrow(() -> new RuntimeException(
                "maple-sim drive not available - is simulation running?"));

        // Create over-the-bumper intake attached to the drive simulation
        intakeSim = IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            driveSim,
            Meters.of(SimConstants.kIntakeWidthMeters),
            Meters.of(SimConstants.kIntakeExtensionMeters),
            IntakeSide.FRONT,
            SimConstants.kIntakeCapacity
        );

        // Register intake with the arena (calls addIntakeSimulation internally)
        intakeSim.register();

        // Place starting fuel on the field
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    /**
     * Steps the arena physics simulation. Must be called every robot periodic cycle.
     */
    public void update() {
        SimulatedArena.getInstance().simulationPeriodic();
    }

    /**
     * Signals the intake to start or stop collecting game pieces.
     * When running, the intake's collision zone is active in the physics engine.
     */
    public void setIntakeRunning(boolean running) {
        if (running) {
            intakeSim.startIntake();
        } else {
            intakeSim.stopIntake();
        }
    }

    /**
     * Checks how many new fuel pieces the intake has collected since last call.
     * Immediately removes them from the intake sim so GamePieceTracker is the
     * single source of truth for held fuel count.
     *
     * @return number of newly obtained fuel pieces (0 if none)
     */
    public int consumeObtainedFuel() {
        int consumed = 0;
        while (intakeSim.getGamePiecesAmount() > 0) {
            if (intakeSim.obtainGamePieceFromIntake()) {
                consumed++;
            } else {
                break;
            }
        }
        return consumed;
    }

    /**
     * Launches a fuel projectile from the robot's current position.
     * The projectile follows ballistic physics and becomes a field piece on landing.
     *
     * @param shooterRPM  current flywheel RPM
     * @param hoodAngleDeg  hood angle in degrees (launch elevation)
     */
    public void launchFuel(double shooterRPM, double hoodAngleDeg) {
        Pose2d robotPose = swerveSubsystem.getPose();
        ChassisSpeeds chassisSpeeds = swerveSubsystem.getSwerveDrive().getFieldVelocity();

        // Convert flywheel RPM to linear exit velocity
        double launchSpeedMPS = (shooterRPM / 60.0)
            * (2.0 * Math.PI * SimConstants.kShooterWheelRadiusMeters);

        RebuiltFuelOnFly projectile = new RebuiltFuelOnFly(
            robotPose.getTranslation(),
            new Translation2d(SimConstants.kShooterOffsetX, SimConstants.kShooterOffsetY),
            chassisSpeeds,
            robotPose.getRotation(),
            Meters.of(SimConstants.kShooterHeightMeters),
            MetersPerSecond.of(launchSpeedMPS),
            Degrees.of(hoodAngleDeg)
        );

        // Fuel lands back on the field as a field piece (recycling)
        projectile.enableBecomesGamePieceOnFieldAfterTouchGround();

        SimulatedArena.getInstance().addGamePieceProjectile(projectile);
    }

    /**
     * Returns positions of all fuel on the field for AdvantageScope visualization.
     */
    public Pose3d[] getFieldFuelPoses() {
        return SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");
    }
}
