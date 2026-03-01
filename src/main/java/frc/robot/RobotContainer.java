// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Constants.FieldPositions;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.FuelPathCommands;
import frc.robot.commands.RunClimberCommand;
import frc.robot.commands.RunConveyorAndKickerCommand;
import frc.robot.commands.RunConveyorCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.RunKickerCommand;
import frc.robot.commands.SlowHoodMove;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TeleopPathplanner;
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem driveBase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/ORCA2026"));
  private final TeleopPathplanner teleopPathplanner;
  @SuppressWarnings("unused") // periodic() runs vision fusion automatically — no commands needed
  // private final VisionSubsystem visionSubsystem;
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  private final KickerSubsystem kickerSubsystem = new KickerSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  
  private final SendableChooser<Command> autoChooser;

  private final CommandXboxController m_primaryController = new CommandXboxController(Constants.OperatorConstants.kDriverControler);

  // private final CommandJoystick m_secondaryController = new CommandJoystick(Constants.kSecondaryDriverControler);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Instantiate all subsystems that require other subsystems here
    // visionSubsystem = new VisionSubsystem(driveBase);
    teleopPathplanner = new TeleopPathplanner(driveBase);

    DriverStation.silenceJoystickConnectionWarning(true);

    // Configure the trigger bindings (set to false for test bindings)
    boolean useProductionBindings = false;
    if (useProductionBindings) {
      configureBindings();
    } else {
      configureTestBindings();
    }
    configureNamedCommands();

    autoChooser = AutoBuilder.buildAutoChooser(); //pick a default
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  SwerveInputStream controllerInput = SwerveInputStream.of(driveBase.getSwerveDrive(),
                    () -> m_primaryController.getLeftY(),
                    () -> m_primaryController.getLeftX())
                    .deadband(OperatorConstants.kStickDeadband)
                    .allianceRelativeControl(true);

  // Rotations for controller input for different driving commands
  SwerveInputStream regularTurning = controllerInput.copy()
                    .withControllerRotationAxis(() -> m_primaryController.getRightX());

  // Transformations for different driving commands
  // Slow drive input for Carter

  Command defaultDriveCommand = driveBase.driveFieldOriented(regularTurning);
  // Make more driving commands
  
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
    driveBase.setDefaultCommand(defaultDriveCommand);

    // Buttons
    m_primaryController.a().and(m_primaryController.back().negate()).onTrue(Commands.runOnce(() -> intakeSubsystem.setIntakeDeployTarget(IntakeSubsystem.Setpoint.kSafe)));
    m_primaryController.b().and(m_primaryController.back().negate()).onTrue(Commands.runOnce(() -> shooterSubsystem.updateHoodTarget(0)));
    m_primaryController.x().and(m_primaryController.back().negate()).onTrue(Commands.runOnce(() -> intakeSubsystem.setIntakeDeployTarget(IntakeSubsystem.Setpoint.kUp)));
    m_primaryController.y().and(m_primaryController.back().negate()).onTrue(Commands.runOnce(() -> intakeSubsystem.setIntakeDeployTarget(IntakeSubsystem.Setpoint.kDown)));

    m_primaryController.start       (); // Unbound for now
    
    // D pad
    m_primaryController.povUp       ().and(m_primaryController.back().negate()).whileTrue(new RunKickerCommand(kickerSubsystem, 5000));
		m_primaryController.povDown     ().and(m_primaryController.back().negate()).whileTrue(new RunKickerCommand(kickerSubsystem, -2500));
    m_primaryController.povLeft     ().and(m_primaryController.back().negate()).whileTrue(new RunConveyorCommand(conveyorSubsystem, 1000));
		m_primaryController.povRight    ().and(m_primaryController.back().negate()).whileTrue(new RunConveyorCommand(conveyorSubsystem, -1000));

    // Axis/Triggers/Sticks
    m_primaryController.rightBumper ().onTrue(Commands.runOnce(() -> shooterSubsystem.setShooterVelocityTarget(1500)))
                                     .onFalse(Commands.runOnce(() -> shooterSubsystem.setShooterVelocityTarget(0)));
    // m_primaryController.rightTrigger().onTrue(Commands.runOnce(() -> driveBase.setDefaultCommand(teleopPathplanner.getHubCommand(controllerInput))))
    //                                  .onFalse(Commands.runOnce(() -> driveBase.setDefaultCommand(defaultDriveCommand)));

    m_primaryController.leftBumper  ().whileTrue(new RunConveyorAndKickerCommand(conveyorSubsystem, kickerSubsystem, 1000, 5000));
    m_primaryController.leftTrigger ().whileTrue(new RunIntakeCommand(intakeSubsystem, 6500));

    m_primaryController.leftStick   ().onTrue(Commands.runOnce(driveBase::zeroGyro));
    // m_primaryController.rightStick   () Slow speed

    // Toggled button options (active while holding back button)
    m_primaryController.back().and(m_primaryController.a()); 
    m_primaryController.back().and(m_primaryController.b());
    m_primaryController.back().and(m_primaryController.x()); // Shooter - by 50
    m_primaryController.back().and(m_primaryController.y()); // Shooter + by 50

    m_primaryController.back().and(m_primaryController.povUp())       .whileTrue(new RunIntakeCommand(intakeSubsystem, 5000)); // Intake up
    m_primaryController.back().and(m_primaryController.povDown())     .whileTrue(new RunIntakeCommand(intakeSubsystem, 5000)); // Intake down
    m_primaryController.back().and(m_primaryController.povLeft())     .whileTrue(new RunIntakeCommand(intakeSubsystem, 5000)); // Hood up
    m_primaryController.back().and(m_primaryController.povRight())    .whileTrue(new RunIntakeCommand(intakeSubsystem, 5000)); // Hood down

    // m_primaryController.back().and(m_primaryController.leftBumper()); - intake out
    // m_primaryController.back().and(m_primaryController.rightBumper()); - slow shooter vomit

    m_primaryController.back().and(m_primaryController.leftTrigger()) .whileTrue(new RunIntakeCommand(intakeSubsystem, -6500));
    // m_primaryController.back().and(m_primaryController.rightTrigger());

    }

  // Testing buttons
  private void configureTestBindings() {
  	driveBase.setDefaultCommand(defaultDriveCommand);
    
    m_primaryController.a           ().onTrue(Commands.runOnce(() -> shooterSubsystem.increaseShooterVelocity(1)));
    m_primaryController.b           ().onTrue(Commands.runOnce(() -> shooterSubsystem.increaseShooterVelocity(2)));
    m_primaryController.x           ().onTrue(Commands.runOnce(() -> shooterSubsystem.increaseShooterVelocity(3)));
    m_primaryController.y           ().onTrue(Commands.runOnce(() -> shooterSubsystem.increaseShooterVelocity(4)));

    m_primaryController.back        ().onTrue(Commands.runOnce(driveBase::zeroGyro));
    m_primaryController.start       ().onTrue(Commands.runOnce(() -> driveBase.setDefaultCommand(teleopPathplanner.getHubDriveCommand(controllerInput))))
                                     .onFalse(Commands.runOnce(() -> driveBase.setDefaultCommand(defaultDriveCommand)));

    m_primaryController.rightBumper ().onTrue(Commands.runOnce(() -> shooterSubsystem.increaseHoodAngle()));
    m_primaryController.leftBumper  ().onTrue(Commands.runOnce(() -> shooterSubsystem.decreaseHoodAngle()));

		m_primaryController.leftStick		().onTrue(Commands.runOnce(() -> shooterSubsystem.updateHoodTarget(0)));
		m_primaryController.rightStick	().onTrue(Commands.runOnce(() -> shooterSubsystem.updateHoodTarget(30)));
    
    m_primaryController.povUp       ().whileTrue(new RunConveyorAndKickerCommand(conveyorSubsystem, kickerSubsystem, 500, 6500));
		m_primaryController.povDown     ().whileTrue(new RunConveyorAndKickerCommand(conveyorSubsystem, kickerSubsystem, -1000, -1000));
    m_primaryController.povLeft     ().whileTrue(new RunIntakeCommand(intakeSubsystem, 6500));
		m_primaryController.povRight    ().whileTrue(new RunIntakeCommand(intakeSubsystem, 0));

    m_primaryController.leftTrigger (0.3).whileTrue(FuelPathCommands.fullFuelPath(intakeSubsystem, conveyorSubsystem, kickerSubsystem));

    m_primaryController.rightTrigger(0.3).onTrue (Commands.runOnce(() -> shooterSubsystem.setToggleDirection(true )))
                                                   .onFalse(Commands.runOnce(() -> shooterSubsystem.setToggleDirection(false)));

    
    
    // m_primaryController.a           ().onTrue(Commands.runOnce(() -> shooterSubsystem.setShooterVelocityTarget(0)));
    // m_primaryController.b           ().onTrue(Commands.runOnce(() -> shooterSubsystem.increaseShooterVelocity(1)));
    // m_primaryController.x           ().onTrue(Commands.runOnce(() -> shooterSubsystem.decreaseShooterVelocity()));
    // m_primaryController.y           ().onTrue(Commands.runOnce(() -> shooterSubsystem.setShooterVelocityTarget(500)));

    // // m_primaryController.start       ().onTrue(Commands.runOnce(driveBase::zeroGyro));

    // m_primaryController.leftStick   ().whileTrue(new RunClimberCommand(climberSubsystem, 1000));
    // m_primaryController.rightStick  ().whileTrue(new RunClimberCommand(climberSubsystem, -1000));

    // m_primaryController.start       ().whileTrue(Commands.runOnce(() -> intakeSubsystem.deployIntake(true )));
    // m_primaryController.back        ().whileTrue(Commands.runOnce(() -> intakeSubsystem.deployIntake(false)));

    // // m_primaryController.start       ().whileTrue(Commands.runOnce(() -> intakeSubsystem.setIntakeDeployDutyCycle(750 )));
    // // m_primaryController.back        ().whileTrue(Commands.runOnce(() -> intakeSubsystem.setIntakeDeployDutyCycle(-2000)));
    // m_primaryController.rightBumper ().whileTrue(Commands.runOnce(() -> intakeSubsystem.setIntakeDeployDutyCycle(0)));
    
    
    // m_primaryController.povUp       ().whileTrue(new RunConveyorAndKickerCommand(conveyorSubsystem, kickerSubsystem, 500, 1500));
    // m_primaryController.povDown     ().whileTrue(new RunConveyorAndKickerCommand(conveyorSubsystem, kickerSubsystem, -1000, -1000));
    // m_primaryController.povLeft     ().whileTrue(new RunIntakeCommand(intakeSubsystem, 6000));
    // m_primaryController.povRight    ().whileTrue(new SlowHoodMove(hoodSubsystem));

    
    // // m_primaryController.rightBumper ().onTrue(Commands.runOnce(() -> hoodSubsystem.increaseHoodAngle()));
    // m_primaryController.leftBumper  ().onTrue(Commands.runOnce(() -> hoodSubsystem.decreaseHoodAngle()));

    // // m_primaryController.leftTrigger (0.3).onTrue   (Commands.runOnce(() -> driveBase.setDefaultCommand(defaultDriveCommand)))
    // //                                                .onFalse  (Commands.runOnce(() -> driveBase.setDefaultCommand(hubCenteringDriveCommand)))
    // //                                                .whileTrue(null); // Shooting routine
    // // m_primaryController.rightTrigger(0.3).whileTrue(teleopPathplanner.createTrenchPathCommand(driveBase));

    // m_primaryController.leftTrigger (0.3).onTrue (Commands.runOnce(() -> intakeSubsystem.setIntakeDeployDutyCycle(3000), intakeSubsystem))
    //                                                .onFalse(Commands.runOnce(() -> intakeSubsystem.setIntakeDeployDutyCycle(0), intakeSubsystem));
    // m_primaryController.rightTrigger(0.3).onTrue (Commands.runOnce(() -> intakeSubsystem.setIntakeDeployDutyCycle(-3000), intakeSubsystem))
    //                                                .onFalse(Commands.runOnce(() -> intakeSubsystem.setIntakeDeployDutyCycle(0), intakeSubsystem));

  }

  private void configureNamedCommands() {
    // Pathplanner commands

    // Run Intake
    NamedCommands.registerCommand("Run Intake",           new RunIntakeCommand(intakeSubsystem, 6000));
    NamedCommands.registerCommand("Stop Intake",          Commands.runOnce(() -> intakeSubsystem.setIntakeDutyCycle(0), intakeSubsystem));

    // Deploy Intake
    NamedCommands.registerCommand("Deploy Intake" ,         Commands.runOnce(() -> intakeSubsystem.deployIntake(true )));
    NamedCommands.registerCommand("Retract Intake",         Commands.runOnce(() -> intakeSubsystem.deployIntake(false)));

    // Shoot
    NamedCommands.registerCommand("Stop Shooter",         Commands.runOnce(() -> shooterSubsystem.setShooterVelocityTarget(0)));
    NamedCommands.registerCommand("Run Shooter Low",      Commands.runOnce(() -> shooterSubsystem.setShooterVelocityTarget(ShooterConstants.kVelocityLow)));
    NamedCommands.registerCommand("Run Shooter Medium",   Commands.runOnce(() -> shooterSubsystem.setShooterVelocityTarget(ShooterConstants.kVelocityMedium)));
    NamedCommands.registerCommand("Run Shooter High",     Commands.runOnce(() -> shooterSubsystem.setShooterVelocityTarget(ShooterConstants.kVelocityHigh)));
    
    // Move hood
    NamedCommands.registerCommand("Hood Position Low",    Commands.runOnce(() -> shooterSubsystem.updateHoodTarget(1)));
    NamedCommands.registerCommand("Hood Position Medium", Commands.runOnce(() -> shooterSubsystem.updateHoodTarget(10)));
    NamedCommands.registerCommand("Hood Position High",   Commands.runOnce(() -> shooterSubsystem.updateHoodTarget(20)));

    // Climb

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

  }

  // Subsystem getters for simulation access
  public SwerveSubsystem getSwerveSubsystem() { return driveBase; }
  public ShooterSubsystem getShooterSubsystem() { return shooterSubsystem; }
  public ConveyorSubsystem getConveyorSubsystem() { return conveyorSubsystem; }
  public KickerSubsystem getKickerSubsystem() { return kickerSubsystem; }
  public IntakeSubsystem getIntakeSubsystem() { return intakeSubsystem; }
  public ClimberSubsystem getClimberSubsystem() { return climberSubsystem; }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
