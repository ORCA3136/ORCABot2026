package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.*;

public class Configs {
    
    public static final class ShooterConfigs {
        public static final SparkFlexConfig primaryShooterConfig = new SparkFlexConfig();
        public static final SparkFlexConfig secondaryShooterConfig = new SparkFlexConfig();

               static {
            primaryShooterConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(CurrentConstants.AMP60, CurrentConstants.AMP40);
            secondaryShooterConfig
                .inverted(true)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(CurrentConstants.AMP60, CurrentConstants.AMP40);
            primaryShooterConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
            secondaryShooterConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
            
        }
    }

    public static final class HoodConfigs {
        public static final SparkMaxConfig primaryHoodConfig = new SparkMaxConfig();
        public static final SparkMaxConfig secondaryHoodConfig = new SparkMaxConfig();

                static {
            primaryHoodConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(CurrentConstants.AMP20, CurrentConstants.AMP15);
            secondaryHoodConfig
                .idleMode(IdleMode.kBrake)
                .follow(CanIdConstants.kHoodPrimaryCanId, true)
                .smartCurrentLimit(CurrentConstants.AMP30, CurrentConstants.AMP20);
            primaryHoodConfig.absoluteEncoder
                .positionConversionFactor(HoodConstants.kMotorGearRatio)
                .velocityConversionFactor(HoodConstants.kMotorGearRatio);
            // TODO: TUNE ON ROBOT — verify encoder conversion factor (12x) matches target calculation (26.67x) in HoodSubsystem
            // Position wrapping must stay enabled — the SparkMax PID with absolute encoder
            // requires it for correct error calculation. Disabling it breaks hood position control.
            primaryHoodConfig.closedLoop
                .positionWrappingEnabled(true)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD)
                .outputRange(-0.4, 0.4); // Old was +-0.8
            // primaryHoodConfig.closedLoop.feedForward
            //     .kG(HoodConstants.kG);
        }
    }

    public static final class ConveyorConfigs {
        public static final SparkFlexConfig conveyorMotorConfig = new SparkFlexConfig();
        static {
            conveyorMotorConfig
                .inverted(true)
                .idleMode(IdleMode.kCoast)
                .voltageCompensation(12)
                .smartCurrentLimit(CurrentConstants.AMP60, CurrentConstants.AMP40);
        }
    }

    public static final class KickerConfigs {
        public static final SparkFlexConfig kickerMotorConfig = new SparkFlexConfig();
        static {
            kickerMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .voltageCompensation(12)
                .smartCurrentLimit(CurrentConstants.AMP60, CurrentConstants.AMP40);
        }
    }

    public static final class IntakeConfigs {
        public static final SparkFlexConfig intakeMotorConfig = new SparkFlexConfig();
        public static final SparkMaxConfig intakeDeployMotorConfig = new SparkMaxConfig();        
        
        static {
            intakeMotorConfig
                .inverted(true)
                .idleMode(IdleMode.kCoast)
                .voltageCompensation(12)
                .smartCurrentLimit(CurrentConstants.AMP60, CurrentConstants.AMP40);
            intakeDeployMotorConfig
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12)
                .smartCurrentLimit(CurrentConstants.AMP60, CurrentConstants.AMP40);
            intakeDeployMotorConfig.absoluteEncoder
                .positionConversionFactor(IntakeConstants.kDeployGearRatio)
                .velocityConversionFactor(IntakeConstants.kDeployGearRatio);
            intakeDeployMotorConfig.closedLoop
                .positionWrappingEnabled(false)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD)
                .outputRange(-0.8, 0.8); // Old was +-0.8
            // primaryHoodConfig.closedLoop.feedForward
            //     .kG(HoodConstants.kG);
        }
    }

    public static final class ClimberConfigs {
        public static final SparkFlexConfig climberPrimaryMotor = new SparkFlexConfig();
        public static final SparkFlexConfig climberSecondaryMotor = new SparkFlexConfig();        
        
        static {
            climberPrimaryMotor
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12)
                .smartCurrentLimit(CurrentConstants.AMP40, CurrentConstants.AMP30);
            climberSecondaryMotor
                .idleMode(IdleMode.kBrake)
                .follow(CanIdConstants.kClimberPrimaryCanId, false)
                .smartCurrentLimit(CurrentConstants.AMP40, CurrentConstants.AMP30);
        }
    }

    // Add more configs
}
