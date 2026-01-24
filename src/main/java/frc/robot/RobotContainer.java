// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.RunConveyorCommand;
import frc.robot.commands.RunHoodCommand;
import frc.robot.commands.RunShooterCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  // private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  
  private final SendableChooser<Command> autoChooser;

  private final CommandXboxController m_primaryController = new CommandXboxController(Constants.OperatorConstants.kDriverControler);

  

  // private final CommandJoystick m_secondaryController = new CommandJoystick(Constants.kSecondaryDriverControler);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    // Configure the trigger bindings
    configureBindings();
    configureNamedCommands();

    autoChooser = AutoBuilder.buildAutoChooser(); //pick a default

  }

  SwerveInputStream controllerInput = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                                           () -> m_primaryController.getLeftY(),
                                                           () -> m_primaryController.getLeftX())
                                                           .withControllerRotationAxis(() -> m_primaryController.getRightX())
                                                           .deadband(Constants.OperatorConstants.kStickDeadband)
                                                           .allianceRelativeControl(true);
    
  // Transformations for controller input for different driving commands

  Command defaultDriveCommand = driveBase.driveFieldOriented(controllerInput);
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
    m_primaryController.a().onTrue(Commands.runOnce(() -> shooterSubsystem.setShooterVelocity(0)));
    m_primaryController.b().onTrue(Commands.runOnce(() -> shooterSubsystem.increaseShooterVelocity()));
    m_primaryController.x().onTrue(Commands.runOnce(() -> shooterSubsystem.decreaseShooterVelocity()));
    m_primaryController.y().onTrue(Commands.runOnce(() -> shooterSubsystem.setShooterVelocity(ShooterConstants.kVelocityMax)));
    m_primaryController.povUp().whileTrue(new RunConveyorCommand(conveyorSubsystem, 500, 4000));
    m_primaryController.povDown().whileTrue(new RunConveyorCommand(conveyorSubsystem, -1000, -1000));
    m_primaryController.rightBumper().whileTrue(new RunHoodCommand(shooterSubsystem, -500));
    m_primaryController.rightBumper().whileTrue(new RunHoodCommand(shooterSubsystem, 500));
    // m_primaryController

    // D pad
    // Axis/Triggers
  }

  private void configureNamedCommands() {
    // Pathplanner commands
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /**
   * Use this to pass the limelight command to the main {@link Robot} class.
   *
   * @return the command to run in disabled
   */
  public Command getLLSeedCommand() {
      return null;//visionSubsystem.getLLSeedCommand();
  }

  /**
   * Use this to pass the limelight command to the main {@link Robot} class.
   *
   * @return the command to run in auto and teleop
   */
  public Command getLLInternalCommand() {
      return null;//visionSubsystem.getLLInternalCommand();
  }
}
