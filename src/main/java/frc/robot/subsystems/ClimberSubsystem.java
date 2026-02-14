// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ClimberConfigs;
import frc.robot.Constants.CanIdConstants;


/*
 * 
 * This subsystem is for climbing the tower
 * Including:
 *    Managing any hooks 
 *    Managing any elevators/climbing poles
 * 
 */


public class ClimberSubsystem extends SubsystemBase {

  final SparkFlex climberPrimaryMotor = new SparkFlex(CanIdConstants.kClimberPrimaryCanId, MotorType.kBrushless);
  final SparkFlex climberSecondaryMotor = new SparkFlex(CanIdConstants.kClimberSecondaryCanId, MotorType.kBrushless);

  final RelativeEncoder climberEncoder = climberPrimaryMotor.getEncoder();

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    climberPrimaryMotor.configure(ClimberConfigs.climberPrimaryMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climberSecondaryMotor.configure(ClimberConfigs.climberSecondaryMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setClimberVelocity(double velocity) {
    climberPrimaryMotor.set(velocity / 6500);
  }

  public double getClimberVlocity() {
    return climberEncoder.getVelocity();
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {

  }

  /** This method will be called once per scheduler run during simulation */
  @Override
  public void simulationPeriodic() {

  }
}
