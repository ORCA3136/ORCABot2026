// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkSim;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
// import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.BuiltInAccelerometerSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

/*
 * 
 * This subsystem is for fuel interactions
 * Including:
 *    Intaking/outaking fuel from the floor
 *    Transporting fuel to the shooter
 * 
 */


public class IntakeSubsystem extends SubsystemBase {

  final DCMotor deployDCMotor = DCMotor.getNeoVortex(1);
  final DCMotor intakeDCMotor = DCMotor.getNeoVortex(1);

  SparkFlex intakeMotor = new SparkFlex(CanIdConstants.kIntakeCanId, MotorType.kBrushless);

  SparkFlex intakeDeployPrimaryMotor = new SparkFlex(CanIdConstants.kIntakeDeployPrimaryCanId, MotorType.kBrushless);
  SparkFlex intakeDeploySecondaryMotor = new SparkFlex(CanIdConstants.kIntakeDeploySecondaryCanId, MotorType.kBrushless);

  final SparkSim deployMotorSim = new SparkSim(intakeDeployPrimaryMotor, deployDCMotor);
  final SparkSim intakeMotorSim = new SparkSim(intakeMotor, intakeDCMotor);

  final SingleJointedArmSim intakeDeploySim = new SingleJointedArmSim(
      deployDCMotor, // Motor type
      IntakeConstants.kDeployGearRatio,
      0.01, // Arm moment of inertia - Small value since there are no arm parameters
      0.1, // Arm length (m) - Small value since there are no arm parameters
      Units.degreesToRadians(-90), // Min angle (rad)
      Units.degreesToRadians(90), // Max angle (rad)
      false, // Simulate gravity - Disable gravity for pivot
      Units.degreesToRadians(0) // Starting position (rad)
    );

  final LinearSystem linearIntake = new LinearSystem<>(null, null, null, null);
  final FlywheelSim intakeFlywheelSim = new FlywheelSim(linearIntake, intakeDCMotor, null);

  RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
  RelativeEncoder intakeDeployEncoder = intakeDeployPrimaryMotor.getEncoder();

  NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  NetworkTable intakeTable = networkTable.getTable(NetworkTableNames.Intake.kIntake);
  NetworkTable intakeDeployTable = networkTable.getTable(NetworkTableNames.IntakeDeploy.kIntakeDeploy);

  private boolean intakeDeployed = false;
  private boolean Override = false;


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

  }

  /**
   * @param Velocity is in RPM
   */
  public void setIntakeVelocity(double velocity) {
    intakeMotor.set(velocity / 6500);
  }

  /**
   * @return Velocity in RPM
   */
  public double getIntakeVelocity() {
    return intakeEncoder.getVelocity();
  }

  /**
   * @param Velocity is in RPM
   */
  public void setIntakeDeployVelocity(double velocity) {
    intakeMotor.set(velocity / 6500);
  }

  /**
   * @return Velocity in RPM
   */
  public double getIntakeDeployVelocity() {
    return intakeEncoder.getVelocity();
  }

  public void setIntakePower(double intakePower) {
    intakeMotor.set(intakePower);
  }

  public void isIntakeDown(boolean Override) {
    if (Override == true) intakeDeployed =! intakeDeployed;
  }

  /**
   * Get the current applied voltage.
   * @return Applied voltage
   */
  public double getFlywheelVoltage() {
    return intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
  }
  /**
   * Get the current applied voltage.
   * @return Applied voltage
   */
  public double getDeployVoltage() {
    return intakeDeployPrimaryMotor.getAppliedOutput() * intakeDeployPrimaryMotor.getBusVoltage();
  }

  public void updateNetworkTable() {
    intakeTable.getEntry(NetworkTableNames.Intake.kVelocityRPM)
      .setNumber(getIntakeVelocity());
    intakeDeployTable.getEntry(NetworkTableNames.IntakeDeploy.kVelocityRPM)
      .setNumber(getIntakeDeployVelocity());
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    updateNetworkTable();
  }

  /** This method will be called once per scheduler run during simulation */
  @Override
  public void simulationPeriodic() {

    // Set input voltage from motor controller to simulation
    // Note: This may need to be talonfx.getSimState().getMotorVoltage() as the input
    //pivotSim.setInput(dcMotor.getVoltage(dcMotor.getTorque(pivotSim.getCurrentDrawAmps()), pivotSim.getVelocityRadPerSec()));
    // pivotSim.setInput(getVoltage());
    // Set input voltage from motor controller to simulation
    // Use getVoltage() for other controllers
    intakeFlywheelSim.setInput(getFlywheelVoltage());
    intakeDeploySim.setInput(getDeployVoltage());

    // Update simulation by 20ms
    intakeFlywheelSim.update(0.020);
    intakeDeploySim.update(0.020);
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(
        intakeFlywheelSim.getCurrentDrawAmps() + 
        intakeDeploySim.getCurrentDrawAmps()
      )
    );

    // double motorDeployPosition = Radians.of(intakeDeploySim.getAngleRads() * IntakeConstants.kDeployGearRatio).in(
    //   Rotations
    // );
    double motorDeployVelocity = RadiansPerSecond.of(
      intakeDeploySim.getVelocityRadPerSec() * IntakeConstants.kDeployGearRatio
    ).in(RotationsPerSecond);
    deployMotorSim.iterate(motorDeployVelocity * 60, RoboRioSim.getVInVoltage(), 0.02);

    // double motorIntakePosition = Radians.of(intakeDeploySim.getAngleRads() * IntakeConstants.kDeployGearRatio).in(
    //   Rotations
    // );
    double motorIntakeVelocity = RadiansPerSecond.of(
      intakeDeploySim.getVelocityRadPerSec() * IntakeConstants.kDeployGearRatio
    ).in(RotationsPerSecond);
    deployMotorSim.iterate(motorIntakeVelocity * 60, RoboRioSim.getVInVoltage(), 0.02);
  }
}
