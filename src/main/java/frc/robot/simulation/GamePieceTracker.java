package frc.robot.simulation;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DIOSim;

/**
 * Tracks fuel through the robot pipeline: INTAKE -> CONVEYOR -> STAGED -> LAUNCHED.
 * Manages a simulated beam break sensor (DIOSim) that trips when fuel is staged
 * at the kicker position, ready for the shooter.
 *
 * Fuel acquisition is driven externally via acquireFuel() (e.g. when intake is
 * running and the robot is near game pieces on the field).
 */
public class GamePieceTracker {

    public enum FuelState {
        EMPTY,      // no fuel in pipeline
        IN_INTAKE,  // fuel acquired, in intake
        IN_CONVEYOR,// fuel moving through conveyor
        STAGED,     // fuel at beam break, ready to shoot
        LAUNCHING   // fuel being fed to shooter
    }

    private final DIOSim beamBreakSim;

    private FuelState state = FuelState.EMPTY;
    private boolean launchRequested = false;
    private int heldFuelCount = 0;
    private int totalLaunched = 0;

    private final NetworkTable simTable;

    public GamePieceTracker() {
        this.beamBreakSim = new DIOSim(SimConstants.kBeamBreakDIOPort);
        beamBreakSim.setValue(true); // beam not broken = no fuel

        simTable = NetworkTableInstance.getDefault().getTable("Simulation");
    }

    /**
     * Simulate acquiring a fuel game piece (e.g. intake picked one up).
     * Only acquires if under capacity.
     */
    public void acquireFuel() {
        if (heldFuelCount < SimConstants.kIntakeCapacity) {
            heldFuelCount++;
            if (state == FuelState.EMPTY) {
                state = FuelState.IN_INTAKE;
            }
        }
    }

    /**
     * Update the fuel pipeline state machine.
     * @param intakeRunning true if intake rollers are spinning
     * @param conveyorForward true if conveyor is running forward
     * @param kickerForward true if kicker is running forward
     * @param shooterAtSpeed true if shooter flywheel is at target speed
     * @return true if a projectile should be launched this cycle
     */
    public boolean update(boolean intakeRunning, boolean conveyorForward,
                          boolean kickerForward, boolean shooterAtSpeed) {
        launchRequested = false;

        switch (state) {
            case EMPTY:
                // Waiting for acquireFuel() to be called externally
                break;

            case IN_INTAKE:
                // Conveyor running moves fuel from intake to conveyor
                if (conveyorForward) {
                    state = FuelState.IN_CONVEYOR;
                }
                break;

            case IN_CONVEYOR:
                // Kicker running stages fuel at beam break position
                if (kickerForward) {
                    state = FuelState.STAGED;
                    beamBreakSim.setValue(false); // beam broken = fuel present
                }
                break;

            case STAGED:
                // Shooter at speed + kicker feeding = launch
                if (shooterAtSpeed && kickerForward) {
                    state = FuelState.LAUNCHING;
                }
                break;

            case LAUNCHING:
                // Consume the fuel and signal projectile launch
                heldFuelCount = Math.max(0, heldFuelCount - 1);
                beamBreakSim.setValue(true); // beam unbroken
                launchRequested = true;
                totalLaunched++;
                // Check if more fuel is available to cycle
                state = heldFuelCount > 0 ? FuelState.IN_INTAKE : FuelState.EMPTY;
                break;
        }

        publishNT();
        return launchRequested;
    }

    private void publishNT() {
        simTable.getEntry("Kicker/BeamBreakTripped").setBoolean(state == FuelState.STAGED || state == FuelState.LAUNCHING);
        simTable.getEntry("Robot/HeldFuelCount").setInteger(heldFuelCount);
        simTable.getEntry("Robot/FuelState").setString(state.name());
        simTable.getEntry("Robot/TotalLaunched").setInteger(totalLaunched);
    }

    public FuelState getState() {
        return state;
    }

    public int getHeldFuelCount() {
        return heldFuelCount;
    }

    public boolean isBeamBreakTripped() {
        return state == FuelState.STAGED || state == FuelState.LAUNCHING;
    }
}
