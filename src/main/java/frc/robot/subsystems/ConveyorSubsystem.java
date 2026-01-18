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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;


/*
 * 
 * This subsystem is for fuel interactions
 * Including:
 *    Intaking/outaking fuel from the floor
 *    Transporting fuel to the shooter
 * 
 */


public class ConveyorSubsystem extends SubsystemBase {

  SparkFlex conveyorMotor = new SparkFlex(Constants.CanIdConstants.kConveyorCanId, MotorType.kBrushless);

  RelativeEncoder conveyorEncoder = conveyorMotor.getEncoder();

  NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  NetworkTable conveyorTable = networkTable.getTable(Constants.NetworkTableNames.Conveyor.kConveyor);

  public ConveyorSubsystem() {

    conveyorMotor.configure(Configs.ConveyorConfigs.conveyorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  /**
   * @param Velocity is in RPM
   */
  public void setConveyorVelocity(double velocity) {
    conveyorMotor.set(velocity);
  }

  /**
   * @return Velocity in RPM
   */
  public double getConveyorVelocity() {
    return conveyorEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    conveyorTable.getEntry(Constants.NetworkTableNames.Conveyor.kVelocityRPM)
      .setNumber(getConveyorVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
