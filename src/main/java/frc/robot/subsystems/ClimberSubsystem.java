// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ClimberConfigs;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.NetworkTableNames;
import frc.robot.Constants.RobotConstants;


/*
 * 
 * This subsystem is for climbing the tower
 * Including:
 *    Managing any hooks 
 *    Managing any elevators/climbing poles
 * 
 */


public class ClimberSubsystem extends SubsystemBase {

  private final SparkFlex climberPrimaryMotor = new SparkFlex(CanIdConstants.kClimberPrimaryCanId, MotorType.kBrushless);
  final SparkFlex climberSecondaryMotor = new SparkFlex(CanIdConstants.kClimberSecondaryCanId, MotorType.kBrushless);

  private final RelativeEncoder climberEncoder = climberPrimaryMotor.getEncoder();

  // private final SparkClosedLoopController = ClimberPIDController Climber

  private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  private final NetworkTable climberTable = networkTable.getTable(NetworkTableNames.Climber.kTable);

  // Cached NetworkTable entries — avoids hash lookups every cycle (50Hz)
  private final NetworkTableEntry velocityEntry = climberTable.getEntry(NetworkTableNames.Climber.kVelocityRPM);
  private final NetworkTableEntry positionEntry = climberTable.getEntry(NetworkTableNames.Climber.kPositionRotations);
  private final NetworkTableEntry primaryCurrentEntry = climberTable.getEntry(NetworkTableNames.Climber.kPrimaryCurrent);
  private final NetworkTableEntry secondaryCurrentEntry = climberTable.getEntry(NetworkTableNames.Climber.kSecondaryCurrent);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    climberPrimaryMotor.configure(ClimberConfigs.climberPrimaryMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climberSecondaryMotor.configure(ClimberConfigs.climberSecondaryMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  /** @return Primary motor for simulation access */
  public SparkFlex getPrimaryMotor() {
    return climberPrimaryMotor;
  }

  /**
   * Sets climber motor output. Takes an RPM-scale value and normalizes it to [-1, 1]
   * duty cycle by dividing by the NEO Vortex free speed (6500 RPM).
   * Only commands the primary motor — the secondary is a follower.
   * @param speed RPM-scale value (e.g. 1000 for extending, -1000 for retracting)
   */
  public void setClimberDutyCycle(double speed) {
    climberPrimaryMotor.set(speed / RobotConstants.kNeoVortexFreeSpeedRPM);
  }

  public double getMotorVelocity() {
    return climberEncoder.getVelocity();
  }

  public double getClimberVelocity() {
    return climberEncoder.getVelocity() / ClimberConstants.kClimberGearRatio;
  }

  public double getMotorRotations() {
    return climberEncoder.getPosition();
  }

  public double getClimberPosition() {
    return climberEncoder.getPosition() / ClimberConstants.kClimberGearRatio;
  }

  public double getClimberPrimaryCurrent() {
    return climberPrimaryMotor.getOutputCurrent();
  }

  public double getClimberSecondaryCurrent() {
    return climberSecondaryMotor.getOutputCurrent();
  }

  public void updateNetworkTable() {
    velocityEntry.setDouble(getClimberVelocity());
    positionEntry.setDouble(getClimberPosition());
    primaryCurrentEntry.setDouble(getClimberPrimaryCurrent());
    secondaryCurrentEntry.setDouble(getClimberSecondaryCurrent());
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {
    updateNetworkTable();
  }

  /** This method will be called once per scheduler run during simulation */
  @Override
  public void simulationPeriodic() {

  }
}
