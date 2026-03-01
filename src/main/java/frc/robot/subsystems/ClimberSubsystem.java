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
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ClimberConfigs;
import frc.robot.Constants.CanIdConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.NetworkTableNames;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.IntakeSubsystem.Setpoint;


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
  private final SparkFlex climberSecondaryMotor = new SparkFlex(CanIdConstants.kClimberSecondaryCanId, MotorType.kBrushless);

  private final AbsoluteEncoder climberEncoder = climberSecondaryMotor.getAbsoluteEncoder();

  private final SparkClosedLoopController ClimberPIDController = climberSecondaryMotor.getClosedLoopController();

  private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  private final NetworkTable climberTable = networkTable.getTable(NetworkTableNames.Climber.kTable);

  // Cached NetworkTable entries — avoids hash lookups every cycle (50Hz)
  private final NetworkTableEntry velocityEntry = climberTable.getEntry(NetworkTableNames.Climber.kVelocityRPM);
  private final NetworkTableEntry positionEntry = climberTable.getEntry(NetworkTableNames.Climber.kPositionRotations);
  private final NetworkTableEntry targetEntry = climberTable.getEntry(NetworkTableNames.Climber.kTargetRotations);
  private final NetworkTableEntry primaryCurrentEntry = climberTable.getEntry(NetworkTableNames.Climber.kPrimaryCurrent);
  private final NetworkTableEntry secondaryCurrentEntry = climberTable.getEntry(NetworkTableNames.Climber.kSecondaryCurrent);

  private double climberTarget = 200; // Temporary
  
  private enum Setpoint {
    kRetracted,
    kL1Climb;
  }

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    climberPrimaryMotor.configure(ClimberConfigs.climberPrimaryMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climberSecondaryMotor.configure(ClimberConfigs.climberSecondaryMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public double calculateFeedForward() {
    // FF pivot = Ksta + Kvel * TarVel + Kgrav * cos(angle) + Kaccel * RobAccel * sin(angle)
    return IntakeConstants.kS + IntakeConstants.kG * Math.cos(getClimberAngle());
  }

  public void setPIDAngle() {
    ClimberPIDController.setSetpoint(climberTarget, ControlType.kPosition, ClosedLoopSlot.kSlot0, calculateFeedForward());
  }

  public void setClimberTarget(Setpoint setpoint) {
    if (setpoint == Setpoint.kRetracted) {
      climberTarget = ClimberConstants.kClimberMinPosition;
    } else if (setpoint == Setpoint.kL1Climb){
      climberTarget = ClimberConstants.kClimberMaxPosition;
    }
  }

  /* sets the climber's target to it's current position */
  public void stopClimber() {
    climberTarget = getClimberPosition();
  }

  /** @param target is in rotations */
  public void setClimberTarget(double target) {
    climberTarget = target;
  }

  public void increaseClimberTarget() {
    climberTarget += ClimberConstants.kClimberIncrement;
  }

  public void decreaseClimberTarget() {
    climberTarget -= ClimberConstants.kClimberIncrement;
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

  /** @return Angle in radians relative to horizontal (0 = horizontal, positive = above, negative = below) 
   * offset commented out, may need to measure later*/
  public double getClimberAngle() {
    return 2 * Math.PI * (climberEncoder.getPosition() /*- ClimberConstants.kClimberOffset */ );
  }

  public double getMotorVelocity() {
    return climberEncoder.getVelocity();
  }

  public double getClimberTarget() {
    return climberTarget;
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
    targetEntry.setDouble(climberTarget);
    primaryCurrentEntry.setDouble(getClimberPrimaryCurrent());
    secondaryCurrentEntry.setDouble(getClimberSecondaryCurrent());
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() {

    setPIDAngle();

    updateNetworkTable();
  }

  /** This method will be called once per scheduler run during simulation */
  @Override
  public void simulationPeriodic() {

  }
}
