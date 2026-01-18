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
 * This subsystem is for all Shooter operations
 * Including:
 *    logging shooter data
 *    Managing the speeds of the flywheels
 * 
 */


public class ShooterSubsystem extends SubsystemBase {

  SparkFlex shooterPrimaryMotor = new SparkFlex(Constants.CanIdConstants.kShooterPrimaryCanId, MotorType.kBrushless);
  SparkFlex shooterSecondaryMotor = new SparkFlex(Constants.CanIdConstants.kShooterSecondaryCanId, MotorType.kBrushless);

  RelativeEncoder shooterEncoder = shooterPrimaryMotor.getEncoder();

  NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  NetworkTable shooterTable = networkTable.getTable(Constants.NetworkTableNames.Shooter.kShooter);


  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {

    shooterPrimaryMotor.configure(Configs.ShooterConfigs.primaryMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterSecondaryMotor.configure(Configs.ShooterConfigs.secondaryMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  /**
   * @param Velocity is in RPM
   */
  public void setShooterVelocity(double velocity) {
    shooterPrimaryMotor.set(velocity);
  }

  /**
   * @return Velocity in RPM
   */
  public double getShooterVelocity() {
    return shooterEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterTable.getEntry(Constants.NetworkTableNames.Shooter.kVelocityRPM)
      .setNumber(getShooterVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
