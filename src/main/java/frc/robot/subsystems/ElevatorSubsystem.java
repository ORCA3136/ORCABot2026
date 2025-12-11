// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants.ElevatorSetpoints;

public class ElevatorSubsystem extends SubsystemBase {

  private VisionSubsystem vision;

  /** Instantiates elevator motors */
  SparkMax leftElevator = new SparkMax(Constants.SparkConstants.kLeftElevatorCanId, MotorType.kBrushless);
  SparkMax rightElevator = new SparkMax(Constants.SparkConstants.kRightElevatorCanId, MotorType.kBrushless);

  SparkMax wristMotor = new SparkMax(Constants.SparkConstants.kWristCanId, MotorType.kBrushless);

  public enum Setpoint {
    kFeederStation,
    kProcessor,
    kLevel2,
    kLevel3,
    kLevel4,
    kTop,
    kBarge,
    kUnblock,
    kTopAlgae,
    kBottomAlgae;
  }

  private Setpoint currentLevel = ElevatorSubsystem.Setpoint.kFeederStation;

  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkClosedLoopController elevatorClosedLoopController =
      leftElevator.getClosedLoopController();

  private SparkClosedLoopController wristClosedLoopController =
      wristMotor.getClosedLoopController();

  private RelativeEncoder elevatorEncoder = leftElevator.getEncoder(); //need a relative encoder to get number of ticks

  private AbsoluteEncoder wristEncoder = wristMotor.getAbsoluteEncoder();
  // Member variables for subsystem state management
  private boolean elevatorReset = false;
  private double elevatorCurrentTarget = Constants.ElevatorConstants.ElevatorSetpoints.kFeederStation;
  private double wristCurrentTarget = Constants.ElevatorConstants.ElevatorSetpoints.kFeederStation;
  
  private boolean changedLevel = false;
  private Setpoint targetSetpoint = Setpoint.kFeederStation;
  private static double distanceToReef = 10;
  private boolean aboveLevel1 = false;
  private boolean manualMode = false;

  private boolean wristManuallyMoving = true;
  private boolean elevatorManuallyMoving = true;

  private final DigitalInput elevatorLimitSwitch;
  private final DigitalInput algaeLimitSwitch;

  
  public ElevatorSubsystem(VisionSubsystem vision) {

    this.vision = vision;

    zeroElevator();
    Configs.ElevatorConfigs.rightElevatorConfig 
          .follow(leftElevator, true);
    
    leftElevator.configure(Configs.ElevatorConfigs.leftElevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rightElevator.configure(Configs.ElevatorConfigs.rightElevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    wristMotor.configure(Configs.WristConfigs.wristMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    elevatorLimitSwitch = new DigitalInput(0);
    algaeLimitSwitch = new DigitalInput(1);
    
  }
  
  // main elevator/wrist movement
    private double getTargetOffset(double target, double pos, double offset, double tolerance){
    if (MathUtil.isNear(target, pos, tolerance)) {
      return target;
    }
    return target + (offset * Math.signum(target - pos)); 
  }

  private double getWristOffset(double target, double offset, double tolerance){
    return getTargetOffset(target, getWristPosition(), offset, tolerance);
  }

  /** Set the elevator motor power in the range of [-1, 1]. */
  public void setElevatorPower(double power) {
    leftElevator.set(power);
    setElevatorManuallyMoving(true);
  }
  
  public void setWristPower(double power) {
    wristMotor.set(power);
    setWristManuallyMoving(true);
  }

  public double getWristCurrentTarget() {
    return wristCurrentTarget;
  }

  public double getElevatorCurrentTarget() {
    return elevatorCurrentTarget;
  }

  private boolean isSetpointAlgae(Setpoint level) {
    if (level == Setpoint.kBarge || level == Setpoint.kTop || level == Setpoint.kBottomAlgae || level == Setpoint.kTopAlgae) {
      return true;
    }
    return false;
  }

  public boolean isCurrentSetpointAlgae() {
    return isSetpointAlgae(currentLevel);
  }

    private void wristMoveToSetpoint() {
    // wristPID(wristCurrentTarget);
    wristClosedLoopController.setReference(
        wristCurrentTarget, ControlType.kPosition);
  }

  private void elevatorMoveToSetpoint() {
    elevatorClosedLoopController.setReference(
      elevatorCurrentTarget, ControlType.kPosition);
  }

  private void wristMoveToSetpoint(double pos) {
    // wristPID(wristCurrentTarget);
    wristClosedLoopController.setReference(
        pos, ControlType.kPosition);
  } 

  private void elevatorMoveToSetpoint(double pos) {
    elevatorClosedLoopController.setReference(
      pos, ControlType.kPosition);
  }
  
  public void updateMode() {
    if (!manualMode) {
      if (distanceToReef > Constants.FieldPoses.reefElevatorRange) {
        targetSetpoint = Setpoint.kFeederStation;
      }
    }
    manualMode = !manualMode;
  }

  public void setMode(boolean bool) {
    manualMode = bool;
  }
  
  public static void updateDistanceToReef(double distance) {
    distanceToReef = distance;
    NetworkTableInstance.getDefault().getTable("Elevator").getEntry("Distance To Reef").setNumber(distance);
  }

  public static double getDistanceToReef() {
    return distanceToReef;
  }

  public Setpoint getSetpoint() {
    return currentLevel;
  }

  public void setWristTarget(double target) {
    wristCurrentTarget = target;
    setWristManuallyMoving(false);
  }

  public void setWristManuallyMoving(boolean bool) {
    wristManuallyMoving = bool;
  }
  
  public boolean isWristManuallyMoving() {
    return wristManuallyMoving;
  }

  public double getWristPosition() {
    return wristEncoder.getPosition(); // 
  }

  public void setElevatorTarget(double target) {
    elevatorCurrentTarget = target;
    setElevatorManuallyMoving(false);
  }
  
  public void setElevatorManuallyMoving(boolean bool) {
    elevatorManuallyMoving = bool;
  }

  public boolean isElevatorManuallyMoving() {
    return elevatorManuallyMoving;
  }

  public double getElevatorPosition() {
    return elevatorEncoder.getPosition();// might need to be scaled by the gear ratio
  }

   /** Zero the elevator encoder when the limit switch is pressed. */
   private void zeroElevatorOnLimitSwitch() {
    if ( (getElevatorPosition() < 0 || !elevatorReset) && !elevatorLimitSwitch.get()) {
      // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
      // prevent constant zeroing while pressed
      // new PrintCommand("RESETTING ELEVATOR");
      zeroElevator();    
      elevatorReset = true;  
    } else if (elevatorLimitSwitch.get()) {
      elevatorReset = false;
    }
  }

  public boolean hasAlgae() {
    return algaeLimitSwitch.get();
  }

  public boolean isElevatorDown() {
    return !elevatorLimitSwitch.get();
  }

  public void zeroElevator() {
    elevatorEncoder.setPosition(0);
  }

    /**
   * A trigger for when the height is at an acceptable tolerance.
   *
   * @param height    Height in encoder units
   * @param tolerance Tolerance in units.
   * @return {@link Trigger}
   */
  public Trigger atHeight(double height, double tolerance)
  {
    return new Trigger(() -> MathUtil.isNear(height,
                                             elevatorEncoder.getPosition(),
                                             tolerance));
  }

  public boolean atHeight() {
    return MathUtil.isNear(elevatorCurrentTarget, getElevatorPosition(), 0.5);
  }

  public boolean atScoringPosition() {
    return MathUtil.isNear(wristCurrentTarget, getWristPosition(), 3);
  }





  public void setTargetSetpoint(Setpoint setpoint) {
    setWristManuallyMoving(false);
    setElevatorManuallyMoving(false);
    targetSetpoint = setpoint;

    switch (targetSetpoint) {
      case kFeederStation:
        aboveLevel1 = false;
        break;
      case kLevel2:
        aboveLevel1 = true;
        break;
      case kLevel3:
        aboveLevel1 = true;
        break;
      case kLevel4:
        aboveLevel1 = true;
        break;
      case kTop:
        aboveLevel1 = true;
        break;
      case kBarge:
        aboveLevel1 = true;
        break;
      case kBottomAlgae:
        aboveLevel1 = true;
        break;
      case kTopAlgae:
        aboveLevel1 = true;
        break;
      case kProcessor:
        aboveLevel1 = false;
        break;
      default:
        break;
    }
  }

  private void updateElevatorHeight() {
    Setpoint newSetpoint;

    if (manualMode) {
      newSetpoint = targetSetpoint;
    } 
    else if (vision.hasCoralInFunnel()) {
      newSetpoint = currentLevel;
    } 
    else if (hasAlgae() && isCurrentSetpointAlgae()) {
      if (isSetpointAlgae(targetSetpoint)) {
        newSetpoint = targetSetpoint;
      } else {
        newSetpoint = currentLevel;
      }
    }
    else {
          if (DriverStation.isAutonomous()) {
            if (distanceToReef < Constants.FieldPoses.reefAutoElevatorRange) {
              newSetpoint = targetSetpoint;
            } else {
              if (aboveLevel1) {
                newSetpoint = Setpoint.kFeederStation;
              } else {
                newSetpoint = targetSetpoint;
              }
            }
          } else {
            if (currentLevel == Setpoint.kBottomAlgae || currentLevel == Setpoint.kTopAlgae) {
              if (distanceToReef < Constants.FieldPoses.reefAlgaeElevatorRange) {
                newSetpoint = targetSetpoint;
              } else {
                if (aboveLevel1) {
                  newSetpoint = Setpoint.kFeederStation;
                } else {
                  newSetpoint = targetSetpoint;
                }
              }
            } else {
              if (distanceToReef < Constants.FieldPoses.reefElevatorRange) {
                newSetpoint = targetSetpoint;
              } else {
                if (aboveLevel1) {
                  newSetpoint = Setpoint.kFeederStation;
                } else {
                  newSetpoint = targetSetpoint;
                }
              }
            }
          }
    }

    NetworkTableInstance.getDefault().getTable("Elevator").getEntry("newSetpoint").setString("" + newSetpoint);
    NetworkTableInstance.getDefault().getTable("Elevator").getEntry("targetSetpoint").setString("" + targetSetpoint);

    if (currentLevel != newSetpoint) {
      setSetpointCommand(newSetpoint);
    }
  }
  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public void setSetpointCommand(Setpoint setpoint) {
    if (currentLevel != setpoint || currentLevel == null) {
      if ( !(isSetpointAlgae(setpoint) && isSetpointAlgae(currentLevel))) 
      {
        changedLevel = true;
      }
    }
    currentLevel = setpoint;

    setWristManuallyMoving(false);
    setElevatorManuallyMoving(false);
    switch (setpoint) {
      case kFeederStation:
        elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
        wristCurrentTarget = Constants.WristConstants.WristSetpoints.kFeederStation;
        break;
      case kLevel2:
        elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
        wristCurrentTarget = Constants.WristConstants.WristSetpoints.kLevel2;
        break;
      case kLevel3:
        elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
        wristCurrentTarget = Constants.WristConstants.WristSetpoints.kLevel3;
        break;
      case kLevel4:
        elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
        wristCurrentTarget = Constants.WristConstants.WristSetpoints.kLevel4;
        break;
      case kTop:
        elevatorCurrentTarget = ElevatorSetpoints.kBarge;
        wristCurrentTarget = Constants.WristConstants.WristSetpoints.kAlgae;
      break;
      case kBarge:
        elevatorCurrentTarget = ElevatorSetpoints.kBarge;
        wristCurrentTarget = Constants.WristConstants.WristSetpoints.kBarge;
        break;
      case kBottomAlgae:
        elevatorCurrentTarget = Constants.ElevatorConstants.ElevatorSetpoints.kBottomAlgae;
        wristCurrentTarget = Constants.WristConstants.WristSetpoints.kAlgae;
        break;
      case kTopAlgae:
        elevatorCurrentTarget = Constants.ElevatorConstants.ElevatorSetpoints.kTopAlgae;
        wristCurrentTarget = Constants.WristConstants.WristSetpoints.kAlgae;
        break;
      case kProcessor:
        elevatorCurrentTarget = Constants.ElevatorConstants.ElevatorSetpoints.kProcessor;
        wristCurrentTarget = Constants.WristConstants.WristSetpoints.kProcessor;
        break;
      case kUnblock:
        wristCurrentTarget = Constants.WristConstants.WristSetpoints.unblock;
        break;
      default:
        break;
    }
  }

  private void moveToSetpointPID() {
    boolean elBool = false;
    double elTarget = 3;

    boolean wristBool = false;
    double wristTarget = 3;


    if (getWristPosition() > Constants.WristConstants.WristSetpoints.unblock + 2 && getWristPosition() < 350) {
      elBool = true;
    } else if (getWristPosition() < Constants.WristConstants.WristSetpoints.unblock - 2 && getElevatorPosition() > Constants.WallConstants.kElevatorAboveTopBar) {
      if (elevatorCurrentTarget < Constants.WallConstants.kElevatorAboveTopBar) { 
        elTarget = Constants.WallConstants.kElevatorAboveTopBar;
      }
    } else if (getWristPosition() > Constants.WristConstants.WristSetpoints.unblock + 2) {
      if (getElevatorPosition() < Constants.WallConstants.kElevatorBelowTopBar) {
        if (elevatorCurrentTarget > Constants.WallConstants.kElevatorBelowTopBar) { 
          elTarget = Constants.WallConstants.kElevatorBelowTopBar;
        } 
      } 
    } else {
      if (elevatorCurrentTarget > Constants.WallConstants.kElevatorBelowBottomBar) {
        elTarget = Constants.WallConstants.kElevatorBelowBottomBar;
      }
    }

    
    if (getElevatorPosition() < Constants.WallConstants.kElevatorBelowBottomBar) {
      wristBool = true;
    } else {
      if (wristCurrentTarget < Constants.WristConstants.WristSetpoints.unblock - 2) {
        wristTarget = Constants.WristConstants.WristSetpoints.unblock;
      }
    }
    
    if (changedLevel) {
      if (Math.abs(getWristPosition() - Constants.WristConstants.WristSetpoints.unblock) < 5) 
        changedLevel = false;
      wristTarget = getWristOffset(Constants.WristConstants.WristSetpoints.unblock, 3, 1);
      wristBool = false;

      elBool = false;
      elTarget = getElevatorPosition();
    }
    else if (Math.abs(getElevatorPosition() - elevatorCurrentTarget) > 0.5) {
      wristBool = false;
      // wristTarget = getWristPosition();
      wristTarget = Constants.WristConstants.WristSetpoints.unblock;
    }

    if (wristCurrentTarget == Constants.WristConstants.WristSetpoints.kAlgae && getWristPosition() > 100) 
    {
      wristBool = true;
      changedLevel = false;
    }


    if (!isElevatorManuallyMoving()) {
      if (elBool) {
        elevatorMoveToSetpoint();
      } else {
        if (elTarget != 3) {
          elevatorMoveToSetpoint(elTarget);
        }
        else {
          elevatorMoveToSetpoint();
        }
      }
    }

    if (!(isWristManuallyMoving())) {
      if (wristBool) {
        wristMoveToSetpoint();
      } else {
        if (wristTarget != 3) {
          wristMoveToSetpoint(wristTarget);
        }
        else {
          wristMoveToSetpoint();
        }
      }
    }

    NetworkTableInstance.getDefault().getTable("Elevator").getEntry("Temp Elevator Target").setNumber(elTarget);
    NetworkTableInstance.getDefault().getTable("Elevator").getEntry("Temp Wrist Target").setNumber(wristTarget);
  }

  @Override
  public void periodic() {
    zeroElevatorOnLimitSwitch();

    if (!wristManuallyMoving && !elevatorManuallyMoving) {
      // Updates wrist and elevator setpoints
      updateElevatorHeight();
    }
    // Controls PID and sets limits
    moveToSetpointPID();

    SmartDashboard.putNumber("Elevator current target", elevatorCurrentTarget);
    SmartDashboard.putNumber("Elevator current position", getElevatorPosition());
    SmartDashboard.putBoolean("Elevator manually moving", wristManuallyMoving);

    SmartDashboard.putNumber("Wrist current target", wristCurrentTarget);
    SmartDashboard.putNumber("Wrist current position", getWristPosition());
    SmartDashboard.putNumber("Wrist current 'angle'", getWristPosition());

    SmartDashboard.putBoolean("Elevator limit switch", !elevatorLimitSwitch.get());
    SmartDashboard.putBoolean("Algaelimit switch", algaeLimitSwitch.get());
    SmartDashboard.putBoolean("Changed Level", changedLevel);

    NetworkTableInstance.getDefault().getTable("Wrist").getEntry("At Scoring Pos").setBoolean(atScoringPosition());
  }

  public void simulationPeriodic() {}
}