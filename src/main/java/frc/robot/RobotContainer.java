// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoScoreCommand;
import frc.robot.commands.DefaultIntakeCommand;
import frc.robot.commands.DoubleLidarRoutine;
import frc.robot.commands.RunClimbSequenceCommand;
import frc.robot.commands.RunClimberCommand;
import frc.robot.commands.RunElevatorCommand;
import frc.robot.commands.RunFunnelCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.RunIntakeScoreCommand;
import frc.robot.commands.RunWristCommand;
import frc.robot.commands.UppercutCommand;
import frc.robot.commands.WaitForCoralCommand;
import frc.robot.commands.ZeroElevatorCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ReefCentering;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ReefCentering.Side;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private VisionSubsystem vision = new VisionSubsystem();
  private final SwerveSubsystem driveBase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/ORCA2025"), vision); // where to configure the robot or "choose" it
  private IntakeSubsystem intake = new IntakeSubsystem();
  private ElevatorSubsystem elevatorSystem = new ElevatorSubsystem(vision);
  private ClimberSubsystem climber = new ClimberSubsystem();
  private LEDSubsystem ledSubsystem = new LEDSubsystem();
  private ReefCentering reefCentering = new ReefCentering(driveBase, elevatorSystem);
  private final SendableChooser<Command> autoChooser;



  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandJoystick m_secondaryController = new CommandJoystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    // Configure the trigger bindings
    configureBindings();
    configureNamedCommands();
    
    autoChooser = AutoBuilder.buildAutoChooser("default auto"); //pick a default
    SmartDashboard.putData("Auto Chooser", autoChooser);

    autoChooser.setDefaultOption("default auto", driveForwardAutoCommand);
  }
    
  SwerveInputStream driveAngularVelocity  = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                          () -> -m_driverController.getLeftY(), 
                                          () -> -m_driverController.getLeftX())
                                          .withControllerRotationAxis(() -> -m_driverController.getRightX())
                                          .deadband(OperatorConstants.DEADBAND)
                                          .scaleTranslation(0.8)
                                          .allianceRelativeControl(true);

  SwerveInputStream driveRegular = driveAngularVelocity.copy().scaleTranslation(1.05);

  SwerveInputStream driveAngularVelocitySlow = driveAngularVelocity.copy().scaleTranslation(Constants.Limits.MEDIUM_SPEED_FACTOR);


  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                                                          .withControllerHeadingAxis(m_driverController::getRightX, 
                                                          m_driverController::getRightY).headingWhile(false);
                                                        //withControllerHeadingAxis(m_driverController::getRightX, m_driverController::getRightX  <- change this to Y for special mode

  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                            .allianceRelativeControl(false);
  SwerveInputStream driveRobotOrientedReefSpeed = driveRobotOriented.copy().scaleTranslation(0.2);
  SwerveInputStream driveRobotOrientedReefFast  = driveRobotOriented.copy().scaleTranslation(2);

  Command driveFieldOrientedDirectAngle = driveBase.driveFieldOriented(driveDirectAngle);
  Command driveFieldOrientedAngularVelocity = driveBase.driveFieldOriented(driveRegular); // Normal Drive
  Command driveFieldOrientedWithElevatorDampening = driveBase.driveFieldOrientedElevatorSpeed(driveRegular, elevatorSystem); // Normal drive with elevator dampening
  Command driveFieldOrientedAngularVelocitySlow = driveBase.driveFieldOriented(driveAngularVelocitySlow); // Right stick
  Command driveRobotOrientedAngularVelocitySuperSlow = driveBase.driveFieldOriented(driveRobotOrientedReefSpeed); // Left stick
  Command driveRobotOrientedAngularVelocitySuperFast = driveBase.driveFieldOriented(driveRobotOrientedReefFast);
  


  SwerveInputStream driveForwardAuto = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                          () -> 0.15, 
                                          () -> 0)
                                          .allianceRelativeControl(false);

  Command driveForwardAutoCommand = driveBase.driveFieldOriented(driveForwardAuto);

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
    driveBase.setDefaultCommand(driveFieldOrientedWithElevatorDampening);
    intake.setDefaultCommand(new DefaultIntakeCommand(intake, vision, climber, elevatorSystem));

    

    m_driverController.start().onTrue(Commands.runOnce(driveBase::zeroGyro));
    m_driverController.back().whileTrue(new ZeroElevatorCommand(elevatorSystem));

    m_driverController.leftStick().whileTrue(driveRobotOrientedAngularVelocitySuperSlow);
    m_driverController.rightStick().whileTrue(driveFieldOrientedAngularVelocitySlow);

    m_driverController.y().whileTrue(new RunElevatorCommand(elevatorSystem, Constants.ElevatorConstants.ElevatorPowerLevels.kUp));
    m_driverController.a().whileTrue(new RunElevatorCommand(elevatorSystem, Constants.ElevatorConstants.ElevatorPowerLevels.kDown));
    m_driverController.b().whileTrue(new RunWristCommand(elevatorSystem, Constants.WristConstants.WristPowerLevels.kOut));
    m_driverController.x().whileTrue(new RunWristCommand(elevatorSystem, Constants.WristConstants.WristPowerLevels.kIn));

    // m_driverController.povDown().whileTrue();
    m_driverController.povUp().whileTrue(reefCentering.createPathCommand(ReefCentering.Side.Middle).until(() -> reefCentering.haveConditionsChanged()).repeatedly());
    m_driverController.povLeft().whileTrue(reefCentering.createPathCommand(ReefCentering.Side.Left).until(() -> reefCentering.haveConditionsChanged()).repeatedly());
    m_driverController.povRight().whileTrue(reefCentering.createPathCommand(ReefCentering.Side.Right).until(() -> reefCentering.haveConditionsChanged()).repeatedly());

    // Right Trigger - 3
    m_driverController.axisGreaterThan(3, 0.4).whileTrue(new RunIntakeCommand(intake, Constants.IntakeConstants.IntakePowerLevels.kOut, vision,ledSubsystem));
    // Left Trigger - 2
    m_driverController.axisGreaterThan(2, 0.4).whileTrue(new RunIntakeScoreCommand(intake, elevatorSystem, ledSubsystem));
      
    m_driverController.rightBumper().whileTrue(new DoubleLidarRoutine(intake, Constants.IntakeConstants.IntakePowerLevels.kFeed, vision, ledSubsystem, climber));
    
    m_driverController.leftBumper().onTrue(new AutoScoreCommand(intake, elevatorSystem, Constants.IntakeConstants.IntakePowerLevels.kOut, vision));

    m_driverController.povDown().whileTrue(reefCentering.createPathCommand(Side.Algae));
    
    m_secondaryController.button(1).whileTrue(Commands.runOnce(() -> elevatorSystem.setTargetSetpoint(ElevatorSubsystem.Setpoint.kLevel4)));
    m_secondaryController.button(2).whileTrue(Commands.runOnce(() -> elevatorSystem.setTargetSetpoint(ElevatorSubsystem.Setpoint.kLevel3)));
    m_secondaryController.button(3).whileTrue(Commands.runOnce(() -> elevatorSystem.setTargetSetpoint(ElevatorSubsystem.Setpoint.kLevel2)));
    m_secondaryController.button(4).whileTrue(Commands.runOnce(() -> elevatorSystem.setTargetSetpoint(ElevatorSubsystem.Setpoint.kFeederStation)));
    m_secondaryController.button(5).whileTrue(Commands.runOnce(() -> elevatorSystem.setTargetSetpoint(ElevatorSubsystem.Setpoint.kTopAlgae)));
    m_secondaryController.button(9).whileTrue(Commands.runOnce(() -> elevatorSystem.setTargetSetpoint(ElevatorSubsystem.Setpoint.kBottomAlgae)));
    m_secondaryController.button(10).whileTrue(Commands.runOnce(() -> {elevatorSystem.setElevatorManuallyMoving(true);
                                                                              elevatorSystem.setWristManuallyMoving(true);
                                                                              elevatorSystem.setElevatorPower(0);
                                                                              elevatorSystem.setWristPower(0); }));
    m_secondaryController.button(6).onTrue(new UppercutCommand(elevatorSystem, intake));
    
    m_secondaryController.button(7).whileTrue(new RunFunnelCommand(climber, Constants.ClimberConstants.kFunnelSpeed));
    m_secondaryController.button(8).whileTrue(new RunClimberCommand(climber, Constants.ClimberConstants.kClimberInSpeed));
    m_secondaryController.button(11).whileTrue(new RunClimberCommand(climber, Constants.ClimberConstants.kClimberOutSpeed));
    m_secondaryController.button(12).whileTrue(Commands.runOnce(() -> elevatorSystem.updateMode()));
    m_secondaryController.axisGreaterThan(0, -0.2).onTrue(new RunClimbSequenceCommand(climber, elevatorSystem, false));
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("Intake score", new RunIntakeCommand(intake, Constants.IntakeConstants.IntakePowerLevels.kOut, vision,ledSubsystem).withTimeout(0.5));
    NamedCommands.registerCommand("Auto score", new AutoScoreCommand(intake, elevatorSystem, 0, vision));
    NamedCommands.registerCommand("Barge score", new UppercutCommand(elevatorSystem, intake));
    NamedCommands.registerCommand("Intake in", new DoubleLidarRoutine(intake, Constants.IntakeConstants.IntakePowerLevels.kFeed, vision, ledSubsystem, climber));
    NamedCommands.registerCommand("Wait for coral", new WaitForCoralCommand(vision));
    NamedCommands.registerCommand("Coral Centering", new DefaultIntakeCommand(intake, vision, climber, elevatorSystem));
    NamedCommands.registerCommand("Elevator Target L1", Commands.runOnce(() -> elevatorSystem.setTargetSetpoint(ElevatorSubsystem.Setpoint.kFeederStation)));
    NamedCommands.registerCommand("Elevator Target L4", Commands.runOnce(() -> elevatorSystem.setTargetSetpoint(ElevatorSubsystem.Setpoint.kLevel4)));
    NamedCommands.registerCommand("Elevator Bottom Algae", Commands.runOnce(() -> elevatorSystem.setTargetSetpoint(ElevatorSubsystem.Setpoint.kBottomAlgae)));
    NamedCommands.registerCommand("Elevator Top Algae", Commands.runOnce(() -> elevatorSystem.setTargetSetpoint(ElevatorSubsystem.Setpoint.kTopAlgae)));
    NamedCommands.registerCommand("Elevator Processor", Commands.runOnce(() -> elevatorSystem.setTargetSetpoint(ElevatorSubsystem.Setpoint.kProcessor)));
    
    // Use coral centering for picking up and holding algae
    // NamedCommands.registerCommand("Hold Algae", Commands.runOnce(() -> elevatorSystem.setElevatorPower(0.1)).handleInterrupt(() -> elevatorSystem.setElevatorPower(0)));
    // NamedCommands.registerCommand("Release Algae", Commands.runOnce(() -> elevatorSystem.setElevatorPower(0.5)).withTimeout(1).handleInterrupt(() -> elevatorSystem.setElevatorPower(0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
