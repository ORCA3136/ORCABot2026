// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfigs;
import frc.robot.Constants.*;

/*
 * This subsystem is for fuel interactions
 * Including:
 *    Intaking/outaking fuel from the floor
 *    Transporting fuel to the shooter
 *
 * === INTAKE DEPLOY PID TUNING STEPS ===
 * 1. Deploy with kG = 0 (current state) — verify both up and down move symmetrically
 * 2. Hold arm perfectly horizontal, read "IntakeDeploy/Position" from NetworkTables
 * 3. Put that value in Constants.java IntakeConstants.kEncoderHorizontalOffset
 * 4. Set kG = 0.05, deploy, and slowly increase until the arm holds position with low current
 *
 * If the arm is too slow in both directions: increase kP (try 1.5, then 3.0)
 * If the arm overshoots and bounces: increase kD (try 3.0) or reduce kP
 * If the arm drifts slowly: add small kI (try 0.001) — use sparingly
 */


public class IntakeSubsystem extends SubsystemBase {

  final DCMotor intakeDCMotor = DCMotor.getNeoVortex(1);
  final DCMotor deployDCMotor = DCMotor.getNeo550(1);

  final SparkFlex intakeMotor = new SparkFlex(CanIdConstants.kIntakeCanId, MotorType.kBrushless);

  final SparkMax intakeDeployMotor = new SparkMax(CanIdConstants.kIntakeDeployCanId, MotorType.kBrushless);

  // final SparkSim deployMotorSim = new SparkSim(intakeDeployMotor, deployDCMotor);
  // final SparkSim intakeMotorSim = new SparkSim(intakeMotor, intakeDCMotor);

  // final SingleJointedArmSim intakeDeploySim = new SingleJointedArmSim(
  //     deployDCMotor, // Motor type
  //     IntakeConstants.kDeployGearRatio,
  //     0.01, // Arm moment of inertia - Small value since there are no arm parameters
  //     0.1, // Arm length (m) - Small value since there are no arm parameters
  //     Units.degreesToRadians(-90), // Min angle (rad)
  //     Units.degreesToRadians(90), // Max angle (rad)
  //     false, // Simulate gravity - Disable gravity for pivot
  //     Units.degreesToRadians(0) // Starting position (rad)
  //   );

  // final LinearSystem linearIntake = new LinearSystem<>(null, null, null, null);
  // final FlywheelSim intakeFlywheelSim = new FlywheelSim(linearIntake, intakeDCMotor, null);

  final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
  final AbsoluteEncoder intakeDeployEncoder = intakeDeployMotor.getAbsoluteEncoder();

  final SparkClosedLoopController IntakePIDController = intakeDeployMotor.getClosedLoopController();

  final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  final NetworkTable intakeTable = networkTable.getTable(NetworkTableNames.Intake.kTable);
  final NetworkTable intakeDeployTable = networkTable.getTable(NetworkTableNames.IntakeDeploy.kTable);

  // Cached NetworkTable entries — avoids hash lookups every cycle (50Hz)
  private final NetworkTableEntry intakeVelocityEntry = intakeTable.getEntry(NetworkTableNames.Intake.kVelocityRPM);
  private final NetworkTableEntry intakeCurrentEntry = intakeTable.getEntry(NetworkTableNames.Intake.kCurrentAmps);
  private final NetworkTableEntry deployCurrentEntry = intakeTable.getEntry(NetworkTableNames.Intake.kDeployCurrentAmps);
  private final NetworkTableEntry deployVelocityEntry = intakeDeployTable.getEntry(NetworkTableNames.IntakeDeploy.kVelocityRPM);
  private final NetworkTableEntry deployPositionEntry = intakeDeployTable.getEntry(NetworkTableNames.IntakeDeploy.kPositionRotations);
  private final NetworkTableEntry deployVoltageEntry = intakeDeployTable.getEntry(NetworkTableNames.IntakeDeploy.kVoltageRotations);

  private boolean intakeDeployed = false;
  private boolean ocillateIntake = false;
  private double ocillationMagnitude = 1;
  private double ocillationFrequency = 1;
  private Setpoint intakeDeployTarget = Setpoint.kUp;

  public enum Setpoint{
    kDown,
    kSafe,
    kUp;
  }


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    intakeMotor.configure(IntakeConfigs.intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeDeployMotor.configure(IntakeConfigs.intakeDeployMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  /** Calculates the current intake feedforward
   * {@summary intake feedforward includes gravitational force, static loss, air resistance, and robot acceleration} */
  public double calculateFeedForward() {
    // FF pivot = Ksta + Kvel * TarVel + Kgrav * cos(angle) + Kaccel * RobAccel * sin(angle)
    return IntakeConstants.kS + IntakeConstants.kG * Math.cos(getIntakeAngle() / IntakeConstants.kDeployGearRatio);
  }

  public double calculatePosition() {

    double tempTargetPosition;

    if (intakeDeployTarget == Setpoint.kDown)
      tempTargetPosition = IntakeConstants.kMinDeployPosition;
    else if (intakeDeployTarget == Setpoint.kSafe)
      tempTargetPosition = IntakeConstants.kSafeDeployPosition;
    else
      tempTargetPosition = IntakeConstants.kMaxDeployPosition;

    // This was adding rotations which would be way too big a change, so I changed it to degrees for the time being
    if (intakeDeployTarget == Setpoint.kUp && ocillateIntake) {
      tempTargetPosition += (ocillationMagnitude * (1 + Math.sin(Timer.getTimestamp() * ocillationFrequency)) / 360);
    }

    if (tempTargetPosition > IntakeConstants.kMaxDeployPosition)
        tempTargetPosition = IntakeConstants.kMaxDeployPosition;
    if (tempTargetPosition < IntakeConstants.kMinDeployPosition)
        tempTargetPosition = IntakeConstants.kMinDeployPosition;

    return tempTargetPosition;
  }

  /** Sets the intake setpoint angle */
  public void setPIDAngle() {
    IntakePIDController.setSetpoint(calculatePosition(), ControlType.kPosition, ClosedLoopSlot.kSlot0, calculateFeedForward());
  }

  /** Updates a boolean used in the calsulatePosition() method, true = deploy */
  public void deployIntake(boolean intakeDeployed) {
    this.intakeDeployed = intakeDeployed;
  }

  /** Sets the intakeDeployTarget variable */
  public void setIntakeDeployTarget(Setpoint position) {
    intakeDeployTarget = position;
  }

  /** @return Intake roller motor for simulation access */
  public SparkFlex getIntakeMotor() {
    return intakeMotor;
  }

  /** @return Deploy motor for simulation access */
  public SparkMax getDeployMotor() {
    return intakeDeployMotor;
  }

  /** @return Angle in radians relative to horizontal (0 = horizontal, positive = above, negative = below) */
  public double getIntakeAngle() {
    return 2 * Math.PI * (intakeDeployEncoder.getPosition() - IntakeConstants.intakeDeployOffset);
  }

  /** True makes the intake ocillate if it's down, false stops it */
  public void ocillateIntake(boolean ocillate) {
    ocillateIntake = ocillate;
  }

  /**
   * Sets intake roller motor output. Takes an RPM-scale value and normalizes it to [-1, 1]
   * duty cycle by dividing by the NEO Vortex free speed (6500 RPM).
   * @param speed RPM-scale value (e.g. 4000 for intaking, -3000 for outtaking)
   */
  public void setIntakeDutyCycle(double speed) {
    intakeMotor.set(speed / RobotConstants.kNeoVortexFreeSpeedRPM);
  }

  /**
   * @return Velocity in RPM
   */
  public double getIntakeVelocity() {
    return intakeEncoder.getVelocity();
  }

  /**
   * Sets intake deploy motor output. Takes an RPM-scale value and normalizes it to [-1, 1]
   * duty cycle by dividing by the NEO 550 free speed (11000 RPM).
   * @param speed RPM-scale value (e.g. 3000 to deploy, -3000 to retract)
   */
  public void setIntakeDeployDutyCycle(double speed) {
    intakeDeployMotor.set(speed / RobotConstants.kNeo550FreeSpeedRPM);
  }

  /**
   * @return Velocity in RPM
   */
  public double getIntakeDeployVelocity() {
    return intakeDeployEncoder.getVelocity();
  }

  /**
   * @return Velocity in RPM
   */
  public double getIntakeDeployPosition() {
    return intakeDeployEncoder.getPosition();
  }

  /**
   * Get the current applied voltage.
   * @return Applied voltage
   */
  public double getVoltage() {
    return intakeDeployMotor.getAppliedOutput() * intakeDeployMotor.getBusVoltage() + 
           intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
  }

  public double getDeployVoltage() {
    return intakeDeployMotor.getAppliedOutput() * intakeDeployMotor.getBusVoltage();
  }

  public void updateNetworkTable() {
    intakeVelocityEntry.setDouble(getIntakeVelocity());
    intakeCurrentEntry.setDouble(intakeMotor.getOutputCurrent());
    deployCurrentEntry.setDouble(intakeDeployMotor.getOutputCurrent());
    deployVelocityEntry.setDouble(getIntakeDeployVelocity());
    deployPositionEntry.setDouble(getIntakeDeployPosition());
    deployVoltageEntry.setDouble(calculateFeedForward());
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {

    updateNetworkTable();



    // TODO: Uncomment setPIDAngle() when deploy PID is tuned on robot
    setPIDAngle();
  }

  /** This method will be called once per scheduler run during simulation */
  @Override
  public void simulationPeriodic() { // Tempotrarily commented out becasue it may be causing problems with the code

    // // Set input voltage from motor controller to simulation
    // // Note: This may need to be talonfx.getSimState().getMotorVoltage() as the input
    // //pivotSim.setInput(dcMotor.getVoltage(dcMotor.getTorque(pivotSim.getCurrentDrawAmps()), pivotSim.getVelocityRadPerSec()));
    // // pivotSim.setInput(getVoltage());
    // // Set input voltage from motor controller to simulation
    // // Use getVoltage() for other controllers
    // intakeFlywheelSim.setInput(getVoltage());
    // intakeDeploySim.setInput(getVoltage());

    // // Update simulation by 20ms
    // intakeFlywheelSim.update(0.020);
    // intakeDeploySim.update(0.020);
    // RoboRioSim.setVInVoltage(
    //   BatterySim.calculateDefaultBatteryLoadedVoltage(
    //     intakeFlywheelSim.getCurrentDrawAmps() + 
    //     intakeDeploySim.getCurrentDrawAmps()
    //   )
    // );

    // // double motorDeployPosition = Radians.of(intakeDeploySim.getAngleRads() * IntakeConstants.kDeployGearRatio).in(
    // //   Rotations
    // // );
    // double motorDeployVelocity = RadiansPerSecond.of(
    //   intakeDeploySim.getVelocityRadPerSec() * IntakeConstants.kDeployGearRatio
    // ).in(RotationsPerSecond);
    // deployMotorSim.iterate(motorDeployVelocity * 60, RoboRioSim.getVInVoltage(), 0.02);

    // // double motorIntakePosition = Radians.of(intakeDeploySim.getAngleRads() * IntakeConstants.kDeployGearRatio).in(
    // //   Rotations
    // // );
    // double motorIntakeVelocity = RadiansPerSecond.of(
    //   intakeDeploySim.getVelocityRadPerSec() * IntakeConstants.kDeployGearRatio
    // ).in(RotationsPerSecond);
    // deployMotorSim.iterate(motorIntakeVelocity * 60, RoboRioSim.getVInVoltage(), 0.02);
  }
}
