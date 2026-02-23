package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Manages Mechanism2d side-view visualization and component Pose3d publishing
 * for AdvantageScope 3D articulated views.
 */
public class MechanismManager {

    // Canvas dimensions (side view of robot)
    private static final double CANVAS_WIDTH = 1.0; // meters
    private static final double CANVAS_HEIGHT = 0.6; // meters

    private final Mechanism2d mechanism;

    // Intake arm ligament
    private final MechanismLigament2d intakeArm;
    // Hood ligament
    private final MechanismLigament2d hoodLigament;
    // Climber ligament
    private final MechanismLigament2d climberLigament;
    // Shooter RPM bar
    private final MechanismLigament2d shooterBar;
    // Fuel staging indicator
    private final MechanismLigament2d fuelIndicator;

    // Component Pose3d publishers for AdvantageScope 3D
    private final NetworkTable componentTable;
    private final StructPublisher<Pose3d> intakeArmPosePublisher;
    private final StructPublisher<Pose3d> shooterHoodPosePublisher;
    private final StructPublisher<Pose3d> climberPosePublisher;

    public MechanismManager() {
        mechanism = new Mechanism2d(CANVAS_WIDTH, CANVAS_HEIGHT);

        // Intake arm: from front of robot, pivots down
        MechanismRoot2d intakeRoot = mechanism.getRoot("Intake", 0.8, 0.15);
        intakeArm = intakeRoot.append(
            new MechanismLigament2d("IntakeArm", 0.2, 0, 6, new Color8Bit(Color.kGray)));

        // Hood: from rear of robot, pivots up
        MechanismRoot2d hoodRoot = mechanism.getRoot("Hood", 0.2, 0.25);
        hoodLigament = hoodRoot.append(
            new MechanismLigament2d("HoodArm", 0.15, 45, 4, new Color8Bit(Color.kYellow)));

        // Climber: extends upward from center
        MechanismRoot2d climberRoot = mechanism.getRoot("Climber", 0.5, 0.15);
        climberLigament = climberRoot.append(
            new MechanismLigament2d("ClimberArm", 0.0, 90, 8, new Color8Bit(Color.kBlue)));

        // Shooter RPM bar: horizontal indicator
        MechanismRoot2d shooterRoot = mechanism.getRoot("Shooter", 0.1, 0.35);
        shooterBar = shooterRoot.append(
            new MechanismLigament2d("ShooterRPM", 0.0, 0, 6, new Color8Bit(Color.kRed)));

        // Fuel staging indicator
        MechanismRoot2d fuelRoot = mechanism.getRoot("Fuel", 0.5, 0.25);
        fuelIndicator = fuelRoot.append(
            new MechanismLigament2d("FuelStage", 0.05, 0, 10, new Color8Bit(Color.kGray)));

        SmartDashboard.putData("Mechanism2d", mechanism);

        // Pose3d publishers
        componentTable = NetworkTableInstance.getDefault().getTable("Simulation/Components");
        intakeArmPosePublisher = componentTable
            .getStructTopic("IntakeArm", Pose3d.struct).publish();
        shooterHoodPosePublisher = componentTable
            .getStructTopic("ShooterHood", Pose3d.struct).publish();
        climberPosePublisher = componentTable
            .getStructTopic("Climber", Pose3d.struct).publish();
    }

    /**
     * Update all mechanism visualizations.
     * @param intakeAngleDeg intake arm angle in degrees (0 = stowed)
     * @param intakeRunning whether intake rollers are spinning
     * @param hoodAngleDeg hood angle in degrees
     * @param shooterRPM current shooter RPM
     * @param shooterTargetRPM target shooter RPM
     * @param climberHeightMeters climber extension in meters
     * @param fuelStaged whether fuel is staged at beam break
     * @param fuelMoving whether fuel is being conveyed
     */
    public void update(double intakeAngleDeg, boolean intakeRunning,
                       double hoodAngleDeg, double shooterRPM, double shooterTargetRPM,
                       double climberHeightMeters, boolean fuelStaged, boolean fuelMoving) {

        // Intake arm: angle and color
        intakeArm.setAngle(-intakeAngleDeg); // negative = deploy downward
        intakeArm.setColor(intakeRunning ? new Color8Bit(Color.kOrange) : new Color8Bit(Color.kGray));

        // Hood: angle and color
        hoodLigament.setAngle(45 + hoodAngleDeg);
        boolean shooterAtSpeed = shooterTargetRPM > 0 && Math.abs(shooterRPM - shooterTargetRPM) < 200;
        hoodLigament.setColor(shooterAtSpeed ? new Color8Bit(Color.kGreen) : new Color8Bit(Color.kYellow));

        // Climber: length proportional to extension
        climberLigament.setLength(climberHeightMeters);

        // Shooter RPM bar: length proportional to RPM/max
        double maxRPM = 6500.0;
        shooterBar.setLength(0.3 * Math.min(shooterRPM / maxRPM, 1.0));
        shooterBar.setColor(shooterAtSpeed ? new Color8Bit(Color.kGreen) : new Color8Bit(Color.kRed));

        // Fuel indicator
        if (fuelStaged) {
            fuelIndicator.setColor(new Color8Bit(Color.kGreen));
        } else if (fuelMoving) {
            fuelIndicator.setColor(new Color8Bit(Color.kYellow));
        } else {
            fuelIndicator.setColor(new Color8Bit(Color.kGray));
        }

        // Publish component Pose3d for AdvantageScope 3D
        intakeArmPosePublisher.set(new Pose3d(
            0.3, 0, 0.15, // robot-relative position
            new Rotation3d(0, Units.degreesToRadians(-intakeAngleDeg), 0)
        ));
        shooterHoodPosePublisher.set(new Pose3d(
            -0.2, 0, 0.25,
            new Rotation3d(0, Units.degreesToRadians(hoodAngleDeg), 0)
        ));
        climberPosePublisher.set(new Pose3d(
            0, 0, 0.15 + climberHeightMeters,
            new Rotation3d()
        ));
    }
}
