// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FuelPathConstants;
import frc.robot.Constants.FieldPositions;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DriveToPositionCommand;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.FuelPathCommands;
import frc.robot.commands.RunConveyorAndKickerCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.TeleopPathplanner;
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  @SuppressWarnings("unused") // periodic() runs vision fusion automatically — no commands needed

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem driveBase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/ORCA2026"));
  // private final TeleopPathplanner teleopPathplanner = new TeleopPathplanner(driveBase);
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(driveBase);
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(driveBase);
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(driveBase);
  private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  private final KickerSubsystem kickerSubsystem = new KickerSubsystem();

  private final SendableChooser<Command> autoChooser;

  private final CommandXboxController m_primaryController = new CommandXboxController(Constants.OperatorConstants.kDriverControler);
  private final CommandJoystick m_secondaryController = new CommandJoystick(OperatorConstants.kSecondaryDriverControler);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    DriverStation.silenceJoystickConnectionWarning(true);

    // Configure the trigger bindings (set to false for test bindings)
    boolean useProductionBindings = true;
    if (useProductionBindings) {
      configureBindings();
    } else {
      configureTestBindings();
    }
    configureOperatorBindings();
    configureNamedCommands();

    autoChooser = AutoBuilder.buildAutoChooser(); //pick a default
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  SwerveInputStream controllerInput = SwerveInputStream.of(driveBase.getSwerveDrive(),
                    () -> driveBase.getAllianceFlip() * -m_primaryController.getLeftY(),
                    () -> driveBase.getAllianceFlip() * -m_primaryController.getLeftX())
                    .deadband(OperatorConstants.kStickDeadband);

  // Deuce: rear-facing shooter — add 180° heading offset so robot aims rear toward hub
  SwerveInputStream aimAtHubStream = controllerInput.copy()
                    .aimWhile(true)
                    .aimHeadingOffset(Rotation2d.fromDegrees(180))
                    .aimHeadingOffset(true);

  // Transformations for different driving commands
  SwerveInputStream slowSpeedDrive   = controllerInput.copy().scaleTranslation(0.4);
  SwerveInputStream mediumSpeedDrive = controllerInput.copy().scaleTranslation(0.8);

  // Rotations for controller input for different driving commands
  SwerveInputStream slowRegularTurning   = slowSpeedDrive  .copy().withControllerRotationAxis(() -> -MathUtil.applyDeadband(m_primaryController.getRightX(), OperatorConstants.kStickDeadband));
  SwerveInputStream mediumRegularTurning = mediumSpeedDrive.copy().withControllerRotationAxis(() -> -MathUtil.applyDeadband(m_primaryController.getRightX(), OperatorConstants.kStickDeadband));
  SwerveInputStream fastRegularTurning   = controllerInput .copy().withControllerRotationAxis(() -> -MathUtil.applyDeadband(m_primaryController.getRightX(), OperatorConstants.kStickDeadband));

  Command slowDriveCommand   = driveBase.driveFieldOriented(slowRegularTurning);
  Command mediumDriveCommand = driveBase.driveFieldOriented(mediumRegularTurning);
  Command fastDriveCommand   = driveBase.driveFieldOriented(fastRegularTurning);

  Command aimAtHubCommand = driveBase.driveFieldOriented(aimAtHubStream);
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driveBase.setDefaultCommand(fastDriveCommand);

    // Buttons
    // Intake deploy percent out
    m_primaryController.a().and(m_primaryController.back().negate()).whileTrue(
        Commands.runEnd(
            () -> intakeSubsystem.setIntakeDeployDutyCycle(650),
            () -> intakeSubsystem.setIntakeDeployDutyCycle(0),
            intakeSubsystem
        ));
    // Intake deploy percent in
    m_primaryController.b().and(m_primaryController.back().negate()).whileTrue(
        Commands.runEnd(
            () -> intakeSubsystem.setIntakeDeployDutyCycle(-650),
            () -> intakeSubsystem.setIntakeDeployDutyCycle(0),
            intakeSubsystem
        ));
    // m_primaryController.b().and(m_primaryController.back().negate()).onTrue(Commands.runOnce(() -> {
    //                                     Translation2d hubPos = driveBase.getAlliance() == DriverStation.Alliance.Red
    //                                         ? FieldPositions.kRedFieldElements.get(0)
    //                                         : FieldPositions.kBlueFieldElements.get(0);
    //                                     aimAtHubStream.aim(new Pose2d(hubPos, new Rotation2d()));
    //                                     Command current = driveBase.getCurrentCommand();
    //                                     if (current != null) current.cancel();
    //                                     driveBase.setDefaultCommand(aimAtHubCommand);
    //                                  }))
    //                                  .onFalse(Commands.runOnce(() -> {
    //                                     Command current = driveBase.getCurrentCommand();
    //                                     if (current != null) current.cancel();
    //                                     driveBase.setDefaultCommand(fastDriveCommand);
    //                                  }));
    // Intake deploy setpoint out
    m_primaryController.x().and(m_primaryController.back().negate())
        .onTrue(Commands.runOnce(() -> intakeSubsystem.setIntakeDeployTarget(IntakeSubsystem.Setpoint.kExtended)));
    // m_primaryController.x().and(m_primaryController.back().negate()).onTrue(Commands.runOnce(() -> intakeSubsystem.setIntakeDeployTarget(IntakeSubsystem.Setpoint.kPartial)));
    // Intake deploy setpoint in
    m_primaryController.y().and(m_primaryController.back().negate())
        .onTrue(Commands.runOnce(() -> intakeSubsystem.setIntakeDeployTarget(IntakeSubsystem.Setpoint.kRetracted)));

    // D pad
    m_primaryController.povUp    ().and(m_primaryController.back().negate()).onTrue(Commands.runOnce(() -> shooterSubsystem.increaseShooterVelocity(4)));
    m_primaryController.povLeft  ().and(m_primaryController.back().negate()).onTrue(Commands.runOnce(() -> shooterSubsystem.increaseShooterVelocity(3)));
		m_primaryController.povRight ().and(m_primaryController.back().negate()).onTrue(Commands.runOnce(() -> shooterSubsystem.increaseShooterVelocity(2)));
    m_primaryController.povDown  ().and(m_primaryController.back().negate()).onTrue(Commands.runOnce(() -> shooterSubsystem.increaseShooterVelocity(1)));

    m_primaryController.back     ().and(m_primaryController.povUp        ()).onTrue(Commands.runOnce(() -> shooterSubsystem.increaseShooterVelocity(-4)));
    m_primaryController.back     ().and(m_primaryController.povLeft      ()).onTrue(Commands.runOnce(() -> shooterSubsystem.increaseShooterVelocity(-3)));
    m_primaryController.back     ().and(m_primaryController.povRight     ()).onTrue(Commands.runOnce(() -> shooterSubsystem.increaseShooterVelocity(-2)));
    m_primaryController.back     ().and(m_primaryController.povDown      ()).onTrue(Commands.runOnce(() -> shooterSubsystem.increaseShooterVelocity(-1)));

    // Axis/Triggers/Sticks

    // m_primaryController.rightBumper().onTrue(Commands.runOnce(() -> shooterSubsystem.setShooterVelocityTarget(1950)))
    //                                  .onFalse(Commands.runOnce(() -> shooterSubsystem.setShooterVelocityTarget(0)));
    // Aim at hub + shoot: auto-rotate rear toward hub while spinning up shooter from distance map
    m_primaryController.rightBumper()
        .onTrue(Commands.runOnce(() -> {
          Translation2d hubPos = driveBase.getAlliance() == DriverStation.Alliance.Red
              ? FieldPositions.kRedFieldElements.get(0)
              : FieldPositions.kBlueFieldElements.get(0);
          aimAtHubStream.aim(new Pose2d(hubPos, new Rotation2d()));
          Command current = driveBase.getCurrentCommand();
          if (current != null) current.cancel();
          driveBase.setDefaultCommand(aimAtHubCommand);
        }))
        .whileTrue(new ShootCommand(shooterSubsystem))
        .onFalse(Commands.runOnce(() -> {
          Command current = driveBase.getCurrentCommand();
          if (current != null) current.cancel();
          driveBase.setDefaultCommand(fastDriveCommand);
        }));
    m_primaryController.rightTrigger()
            .whileTrue(new ShootCommand(shooterSubsystem));

    // Feed: slow retract to pull fuel in, then shuttle pulse for remaining fuel
    m_primaryController.leftBumper  ().whileTrue(Commands.parallel(
        new RunConveyorAndKickerCommand(conveyorSubsystem, kickerSubsystem, 1000, 5000),
        Commands.sequence(
            // Phase 1: slow retract with intake roller running
            Commands.runOnce(() -> intakeSubsystem.slowRetract(true)),
            new RunIntakeCommand(intakeSubsystem, 4000).withTimeout(2),
            // Phase 2: shuttle pulse to clear remaining fuel
            Commands.runOnce(() -> intakeSubsystem.slowRetract(false)),
            FuelPathCommands.intakeShuttlePulse(intakeSubsystem).withTimeout(3),
            Commands.runOnce(() -> intakeSubsystem.setIntakeDeployTarget(IntakeSubsystem.Setpoint.kRetracted))
        )
    ).finallyDo(interrupted -> {
        intakeSubsystem.slowRetract(false);
        intakeSubsystem.pulse(false);
    }));
    m_primaryController.leftTrigger ().whileTrue(new RunIntakeCommand(intakeSubsystem, 6500));

    m_primaryController.leftStick   ().onTrue(Commands.runOnce(driveBase::zeroGyro));
    // Right stick held: medium speed driving (0.8x translation)
    m_primaryController.rightStick  ().whileTrue(Commands.runOnce(() -> {
                                        Command current = driveBase.getCurrentCommand();
                                        if (current != null) current.cancel();
                                        driveBase.setDefaultCommand(slowDriveCommand);
                                     }))
                                     .onFalse(Commands.runOnce(() -> {
                                        Command current = driveBase.getCurrentCommand();
                                        if (current != null) current.cancel();
                                        driveBase.setDefaultCommand(fastDriveCommand);
                                     }));

    // Toggled button options (active while holding back button)
    m_primaryController.back().and(m_primaryController.a())           .onTrue(Commands.runOnce(() -> shooterSubsystem.increaseShooterVelocity(2)));  // Shooter - by 50
    m_primaryController.back().and(m_primaryController.b())           .onTrue(Commands.runOnce(() -> shooterSubsystem.increaseShooterVelocity(-2))); // Shooter + by 50
    m_primaryController.back().and(m_primaryController.x())           .onTrue(Commands.runOnce(() -> intakeSubsystem.setIntakeDeployDutyCycle(2000)))
                                                                      .onFalse(Commands.runOnce(() -> intakeSubsystem.setIntakeDeployDutyCycle(0)));
    m_primaryController.back().and(m_primaryController.y())           .onTrue(Commands.runOnce(() -> intakeSubsystem.setIntakeDeployDutyCycle(-2000)))
                                                                      .onFalse(Commands.runOnce(() -> intakeSubsystem.setIntakeDeployDutyCycle(0)));

    m_primaryController.back().and(m_primaryController.leftTrigger()) .whileTrue(new RunIntakeCommand(intakeSubsystem, -6500));
    // m_primaryController.back().and(m_primaryController.rightTrigger());

    }

 

  /** Operator button board bindings for drive-to-position commands. */
  private void configureOperatorBindings() {
    // Manual climber jog with software limits
    m_secondaryController.button(1);
    m_secondaryController.button(2);
    m_secondaryController.button(3);
    m_secondaryController.button(4);

    m_secondaryController.button(5).whileTrue(Commands.run(() -> driveBase.lockPose(), driveBase));
    // Clear intake fault
    m_secondaryController.button(6).onTrue(Commands.runOnce(() -> {
      intakeSubsystem.clearFault();
      intakeSubsystem.requestHoming();
    }));
    // Prepare to climb — stow intake, move arm to horizontal
    // m_secondaryController.button(6).onTrue(
    //     ClimberCommands.prepareToClimb(climberSubsystem, intakeSubsystem));

    // // Execute climb — pull arm to climbed position and hold
    // m_secondaryController.button(7).onTrue(
    //     ClimberCommands.executeClimb(climberSubsystem));

    // Shoot on the move: aim at lead-compensated hub + full pipeline + moving shooter map
    m_secondaryController.button(8)
        .onTrue(Commands.runOnce(() -> {
          // Seed the lead compensation with an initial computation so the
          // aim supplier returns the real hub position on the first cycle
          shooterSubsystem.setShooterMapLeadCompensated();
          aimAtHubStream.aim(() -> new Pose2d(
              shooterSubsystem.getLeadCompensatedHubTranslation(), new Rotation2d()));
          Command current = driveBase.getCurrentCommand();
          if (current != null) current.cancel();
          driveBase.setDefaultCommand(aimAtHubCommand);
        }))
        .whileTrue(FuelPathCommands.shootOnMove(
            intakeSubsystem, conveyorSubsystem, kickerSubsystem, shooterSubsystem))
        .onFalse(Commands.runOnce(() -> {
          Command current = driveBase.getCurrentCommand();
          if (current != null) current.cancel();
          driveBase.setDefaultCommand(fastDriveCommand);
        }));

    // Scoring position buttons — operator taps to send robot, driver overrides with sticks
    // m_secondaryController.button(9).onTrue(
    //     DriveToPositionCommand.driveToScoreClose(driveBase, shooterSubsystem, this::driverIsOverriding));
    // m_secondaryController.button(10).onTrue(
    //     DriveToPositionCommand.driveToScoreTrench(driveBase, shooterSubsystem, this::driverIsOverriding));
    m_secondaryController.button(9).whileTrue(
        Commands.runEnd(
            () -> intakeSubsystem.setIntakeDeployDutyCycle(1500),
            () -> intakeSubsystem.setIntakeDeployDutyCycle(0),
            intakeSubsystem
        ));
    m_secondaryController.button(10).whileTrue(
        Commands.runEnd(
            () -> intakeSubsystem.setIntakeDeployDutyCycle(-1000),
            () -> intakeSubsystem.setIntakeDeployDutyCycle(0),
            intakeSubsystem
        ));
    //known good scoring position that we like
    m_secondaryController.button(11).onTrue(
        DriveToPositionCommand.driveToScoreCustom(driveBase, shooterSubsystem, this::driverIsOverriding));
  // m_secondaryController.button(12).whileTrue(                                                                                                                                     
  //           Commands.sequence(                                                                                                                                                   
  //                Commands.runOnce(() -> intakeSubsystem.setIntakeDeployTarget(IntakeSubsystem.Setpoint.kRetracted)),                                                                     
  //                Commands.waitUntil(() -> Math.abs(intakeSubsystem.getIntakeDeployPosition() - IntakeConstants.kMaxDeployPosition) < 0.02),                                       
  //                Commands.runOnce(() -> intakeSubsystem.setIntakeDeployTarget(IntakeSubsystem.Setpoint.kPartial)),                                                                   
  //                Commands.waitUntil(() -> Math.abs(intakeSubsystem.getIntakeDeployPosition() - IntakeConstants.kSafeDeployPosition) < 0.02)                                       
  //          ).repeatedly()                                                                                                                                                       
  //      );       
    m_secondaryController.button(12).onTrue(
        Commands.runOnce(() -> intakeSubsystem.setIntakeDeployTarget(IntakeSubsystem.Setpoint.kExtended), intakeSubsystem));
  }

  private void configureNamedCommands() {
    // Pathplanner commands
    NamedCommands.registerCommand("Aim at Hub",           DriveToPositionCommand.aimAtHub(driveBase));

    // Run Intake
    NamedCommands.registerCommand("Run Intake",           new RunIntakeCommand(intakeSubsystem, 6000));
    NamedCommands.registerCommand("Stop Intake",          Commands.runOnce(() -> intakeSubsystem.setIntakeDutyCycle(0), intakeSubsystem));

    // Deploy Intake (with subsystem requirement to prevent conflicts)
    NamedCommands.registerCommand("Deploy Intake" ,       Commands.runOnce(() -> intakeSubsystem.setIntakeDeployTarget(IntakeSubsystem.Setpoint.kExtended), intakeSubsystem));
    NamedCommands.registerCommand("Intake Safe" ,         Commands.runOnce(() -> intakeSubsystem.setIntakeDeployTarget(IntakeSubsystem.Setpoint.kRetracted), intakeSubsystem));
    NamedCommands.registerCommand("Retract Intake",       Commands.runOnce(() -> intakeSubsystem.setIntakeDeployTarget(IntakeSubsystem.Setpoint.kRetracted), intakeSubsystem));

    // Shoot
    NamedCommands.registerCommand("Shoot to Hub",         FuelPathCommands.shootToHub(shooterSubsystem));
    NamedCommands.registerCommand("Stop Shooter",         Commands.runOnce(() -> shooterSubsystem.setShooterVelocityTarget(0)));
    NamedCommands.registerCommand("Run Shooter Low",      Commands.runOnce(() -> shooterSubsystem.setShooterVelocityTarget(ShooterConstants.kVelocityLow)));
    NamedCommands.registerCommand("Run Shooter Medium",   Commands.runOnce(() -> shooterSubsystem.setShooterVelocityTarget(ShooterConstants.kVelocityMedium)));
    NamedCommands.registerCommand("Run Shooter High",     Commands.runOnce(() -> shooterSubsystem.setShooterVelocityTarget(ShooterConstants.kVelocityHigh)));
    
    // Conveyor/kicker
    NamedCommands.registerCommand("Run Conveyor",         new RunConveyorAndKickerCommand(conveyorSubsystem, kickerSubsystem, 500, 4000));
    NamedCommands.registerCommand("Stop Conveyor",        Commands.runOnce(() -> { conveyorSubsystem.setConveyorDutyCycle(0); kickerSubsystem.setKickerDutyCycle(0); }, conveyorSubsystem, kickerSubsystem));

    // Fuel path commands
    NamedCommands.registerCommand("Full Fuel Path",       FuelPathCommands.fullFuelPath(intakeSubsystem, conveyorSubsystem, kickerSubsystem));
    NamedCommands.registerCommand("Stop Fuel Path",       FuelPathCommands.stopAll(intakeSubsystem, conveyorSubsystem, kickerSubsystem));
    NamedCommands.registerCommand("Intake And Conveyor",  FuelPathCommands.intakeAndConveyor(intakeSubsystem, conveyorSubsystem));
    NamedCommands.registerCommand("Metered Feed",         FuelPathCommands.meteredFeed(conveyorSubsystem, kickerSubsystem, shooterSubsystem));
    NamedCommands.registerCommand("Kicker Pulse",         FuelPathCommands.kickerPulse(kickerSubsystem));
    NamedCommands.registerCommand("Emergency Reverse",    FuelPathCommands.emergencyReverseAll(intakeSubsystem, conveyorSubsystem, kickerSubsystem));

    // ── Auto-safe commands (timeout-safe, named, with cleanup) ──────────

    // Instant deploy position commands
    NamedCommands.registerCommand("IntakeDown",            AutoCommands.intakeDown(intakeSubsystem));
    NamedCommands.registerCommand("IntakeSafe",            AutoCommands.intakeUp(intakeSubsystem));
    NamedCommands.registerCommand("IntakeUp",              AutoCommands.intakeUp(intakeSubsystem));

    // Timeout-safe motor commands
    NamedCommands.registerCommand("RunIntake3s",           AutoCommands.runIntake(intakeSubsystem, FuelPathConstants.kIntakeInStandard, 3.0));
    NamedCommands.registerCommand("RunIntake5s",           AutoCommands.runIntake(intakeSubsystem, FuelPathConstants.kIntakeInStandard, 5.0));
    NamedCommands.registerCommand("RunConveyor3s",         AutoCommands.runConveyor(conveyorSubsystem, FuelPathConstants.kConveyorIn, 3.0));
    NamedCommands.registerCommand("RunConveyor5s",         AutoCommands.runConveyor(conveyorSubsystem, FuelPathConstants.kConveyorIn, 5.0));
    NamedCommands.registerCommand("RunKicker2s",           AutoCommands.runKicker(kickerSubsystem, FuelPathConstants.kKickerFeed, 2.0));
    NamedCommands.registerCommand("FeedAll3s",             AutoCommands.feedAll(conveyorSubsystem, kickerSubsystem, FuelPathConstants.kConveyorIn, FuelPathConstants.kKickerFeed, 3.0));
    NamedCommands.registerCommand("FeedAll5s",             AutoCommands.feedAll(conveyorSubsystem, kickerSubsystem, FuelPathConstants.kConveyorIn, FuelPathConstants.kKickerFeed, 5.0));
    NamedCommands.registerCommand("IntakeWithDeploy3s",    AutoCommands.intakeWithDeploy(intakeSubsystem, FuelPathConstants.kIntakeInStandard, 3.0));
    NamedCommands.registerCommand("IntakeWithDeploy5s",    AutoCommands.intakeWithDeploy(intakeSubsystem, FuelPathConstants.kIntakeInStandard, 5.0));

    // Smart shooting commands
    NamedCommands.registerCommand("AimAndShoot",           AutoCommands.aimAndShoot(shooterSubsystem, driveBase, kickerSubsystem, conveyorSubsystem, AutoConstants.kShootTimeoutSec));
    NamedCommands.registerCommand("MeteredFeed3s",         AutoCommands.meteredFeedTimed(conveyorSubsystem, kickerSubsystem, shooterSubsystem, AutoConstants.kFeedTimeoutSec));
    NamedCommands.registerCommand("SpinUpFromDistance",    AutoCommands.spinUpFromDistance(shooterSubsystem, driveBase));

    // Teleop-mirrored sequences (safe for parallel use in PathPlanner)
    NamedCommands.registerCommand("Shoot Sequence",       AutoCommands.shootSequence(shooterSubsystem, 6.0));
    NamedCommands.registerCommand("Feed Sequence",        AutoCommands.feedSequence(intakeSubsystem, conveyorSubsystem, kickerSubsystem, 6.0));

    // Drive-to-position
    NamedCommands.registerCommand("Drive To Hub",          DriveToPositionCommand.driveToHub(driveBase));
    NamedCommands.registerCommand("Drive To Tower",        DriveToPositionCommand.driveToTower(driveBase));
    NamedCommands.registerCommand("Drive To Left Trench",  DriveToPositionCommand.driveToLeftTrench(driveBase));
    NamedCommands.registerCommand("Drive To Right Trench", DriveToPositionCommand.driveToRightTrench(driveBase));
    NamedCommands.registerCommand("Drive To Outpost",      DriveToPositionCommand.driveToOutpost(driveBase));
    NamedCommands.registerCommand("Drive To Depot",        DriveToPositionCommand.driveToDepot(driveBase));
  }

  /** Returns true if the driver is actively pushing sticks (wants manual control). */
  private boolean driverIsOverriding() {
    double db = FieldPositions.kDriverOverrideDeadband;
    return Math.abs(m_primaryController.getLeftX()) > db
        || Math.abs(m_primaryController.getLeftY()) > db
        || Math.abs(m_primaryController.getRightX()) > db;
  }

  // Subsystem getters for simulation access
  public SwerveSubsystem getSwerveSubsystem() { return driveBase; }
  public ShooterSubsystem getShooterSubsystem() { return shooterSubsystem; }
  public ConveyorSubsystem getConveyorSubsystem() { return conveyorSubsystem; }
  public KickerSubsystem getKickerSubsystem() { return kickerSubsystem; }
  public IntakeSubsystem getIntakeSubsystem() { return intakeSubsystem; }
  public VisionSubsystem getVisionSubsystem() { return visionSubsystem; }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

   // Testing buttons
  private void configureTestBindings() {
  	driveBase.setDefaultCommand(fastDriveCommand);
    
    m_primaryController.a           ().onTrue(Commands.runOnce(() -> shooterSubsystem.increaseShooterVelocity(1)));
    m_primaryController.b           ().onTrue(Commands.runOnce(() -> shooterSubsystem.increaseShooterVelocity(2)));
    m_primaryController.x           ().onTrue(Commands.runOnce(() -> shooterSubsystem.increaseShooterVelocity(3)));
    m_primaryController.y           ().onTrue(Commands.runOnce(() -> shooterSubsystem.increaseShooterVelocity(4)));

    m_primaryController.back        ().onTrue(Commands.runOnce(driveBase::zeroGyro));
    m_primaryController.start       ().onTrue(Commands.runOnce(() -> {
                                        Translation2d hubPos = driveBase.getAlliance() == DriverStation.Alliance.Red
                                            ? FieldPositions.kRedFieldElements.get(0)
                                            : FieldPositions.kBlueFieldElements.get(0);
                                        aimAtHubStream.aim(new Pose2d(hubPos, new Rotation2d()));
                                        Command current = driveBase.getCurrentCommand();
                                        if (current != null) current.cancel();
                                        driveBase.setDefaultCommand(aimAtHubCommand);
                                     }))
                                     .onFalse(Commands.runOnce(() -> {
                                        Command current = driveBase.getCurrentCommand();
                                        if (current != null) current.cancel();
                                        driveBase.setDefaultCommand(fastDriveCommand);
                                     }));


    
    m_primaryController.povUp       ().whileTrue(new RunConveyorAndKickerCommand(conveyorSubsystem, kickerSubsystem, 500, 6500));
		m_primaryController.povDown     ().whileTrue(new RunConveyorAndKickerCommand(conveyorSubsystem, kickerSubsystem, -1000, -1000));
    m_primaryController.povLeft     ().whileTrue(new RunIntakeCommand(intakeSubsystem, 6500));
		m_primaryController.povRight    ().whileTrue(new RunIntakeCommand(intakeSubsystem, -6500));

    m_primaryController.leftTrigger (0.3).whileTrue(FuelPathCommands.fullFuelPath(intakeSubsystem, conveyorSubsystem, kickerSubsystem));


  }

}