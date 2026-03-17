package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
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
        public static final SparkFlexConfig intakeDeployMotorConfig = new SparkFlexConfig();

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
                .smartCurrentLimit(CurrentConstants.AMP40, CurrentConstants.AMP30);
            // Disable soft limits at boot — they are applied dynamically after homing
            intakeDeployMotorConfig.softLimit
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimitEnabled(false);
            // Relative encoder — raw motor rotations (no conversion factor needed,
            // PID setpoints are in motor rotations from home)
            intakeDeployMotorConfig.closedLoop
                .positionWrappingEnabled(false)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD)
                .outputRange(-IntakeConstants.kMaxOutputDutyCycle, IntakeConstants.kMaxOutputDutyCycle);
        }
    }

    public static final class ClimberConfigs {
        public static final SparkFlexConfig climberPrimaryMotor = new SparkFlexConfig();

        static {
            // Primary: PID leader with relative encoder
            climberPrimaryMotor
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12)
                .smartCurrentLimit(CurrentConstants.AMP40, CurrentConstants.AMP30);
            climberPrimaryMotor.encoder
                .positionConversionFactor(360.0 / ClimberConstants.kTotalReduction)   // motor rot → arm degrees
                .velocityConversionFactor(360.0 / ClimberConstants.kTotalReduction / 60.0); // RPM → deg/s
            climberPrimaryMotor.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD)
                .outputRange(-0.8, 0.8);
            // Hardware soft limits — backup safety layer on motor controller
            climberPrimaryMotor.softLimit
                .forwardSoftLimitEnabled(true)
                .forwardSoftLimit((float)(ClimberConstants.kMaxArmDegrees - 2.0))
                .reverseSoftLimitEnabled(true)
                .reverseSoftLimit((float)(ClimberConstants.kMinArmDegrees + 2.0));
        }
    }

    // Add more configs
}
