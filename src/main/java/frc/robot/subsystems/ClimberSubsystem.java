// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  SparkMax climberMotor = new SparkMax(Constants.SparkConstants.kClimberCanId, MotorType.kBrushless);
  SparkMax funnelMotor = new SparkMax(Constants.SparkConstants.kFunnelCanId, MotorType.kBrushless);

  private RelativeEncoder climberEncoder = climberMotor.getEncoder();
  private RelativeEncoder funnelEncoder = funnelMotor.getEncoder();

  private boolean startedClimbing = false;


  /** Creates a new ExampleSubsystem. */
  public ClimberSubsystem() {
    funnelMotor.configure(Configs.ClimberConfigs.funnelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climberMotor.configure(Configs.ClimberConfigs.climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  
  public void setClimberPower(double power) {
    climberMotor.set(power);
  }

  public void setFunnelPower(double power) {
    funnelMotor.set(power);
  }

  public double getClimberPosition() {
    return climberEncoder.getPosition();
  }

  public double getFunnelPosition() {
    return funnelEncoder.getPosition();
  }

  public boolean isFlipped() {
    return getFunnelPosition() < Constants.ClimberConstants.kFunnelOutPos;
  }

  public void setClimbingMode(boolean mode) {
    startedClimbing = mode;
  }

  public boolean getClimbingMode() {
    return startedClimbing;
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("funnel position", getFunnelPosition());
    SmartDashboard.putNumber("climber Position", getClimberPosition());
  }

  @Override
  public void simulationPeriodic() {}
}
