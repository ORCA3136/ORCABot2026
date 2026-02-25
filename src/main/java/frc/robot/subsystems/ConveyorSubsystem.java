// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.*;
import frc.robot.Constants.*;


/*
 * 
 * This subsystem is for fuel interactions
 * Including:
 *    Intaking/outaking fuel from the floor
 *    Transporting fuel to the shooter
 * 
 */


public class ConveyorSubsystem extends SubsystemBase {

  final SparkFlex conveyorMotor = new SparkFlex(CanIdConstants.kConveyorCanId, MotorType.kBrushless);

  final RelativeEncoder conveyorEncoder = conveyorMotor.getEncoder();

  final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  final NetworkTable conveyorTable = networkTable.getTable(NetworkTableNames.Conveyor.kTable);

  // Cached NetworkTable entries — avoids hash lookups every cycle (50Hz)
  private final NetworkTableEntry velocityEntry = conveyorTable.getEntry(NetworkTableNames.Conveyor.kVelocityRPM);
  private final NetworkTableEntry currentEntry = conveyorTable.getEntry(NetworkTableNames.Conveyor.kCurrentAmps);

  /** Creates a new ConveyorSubsystem. */
  public ConveyorSubsystem() {

    conveyorMotor.configure(ConveyorConfigs.conveyorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  /** @return Motor for simulation access */
  public SparkFlex getMotor() {
    return conveyorMotor;
  }

  /**
   * Sets conveyor motor output. Takes an RPM-scale value and normalizes it to [-1, 1]
   * duty cycle by dividing by the NEO Vortex free speed (6500 RPM).
   * Example: setConveyorDutyCycle(3250) → motor runs at ~50% power.
   * @param speed RPM-scale value (e.g. 500 for slow forward, -1000 for reverse)
   */
  public void setConveyorDutyCycle(double speed) {
    conveyorMotor.set(speed / RobotConstants.kNeoVortexFreeSpeedRPM);
  }

  /**
   * @return Velocity in RPM
   */
  public double getConveyorVelocity() {
    return conveyorEncoder.getVelocity();
  }

  /**
   * @return Current is in Amps
   */
  public double getConveyorCurrent() {
    return conveyorMotor.getOutputCurrent();
  }

  public void updateNetworkTable() {
    velocityEntry.setDouble(getConveyorVelocity());
    currentEntry.setDouble(getConveyorCurrent());
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
