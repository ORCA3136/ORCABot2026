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
                .inverted(true)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(CurrentConstants.AMP60, CurrentConstants.AMP40);
            secondaryShooterConfig
                .inverted(false)
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
                .idleMode(IdleMode.kCoast)
                .voltageCompensation(12)
                .smartCurrentLimit(CurrentConstants.AMP40, CurrentConstants.AMP30);
            // Disable soft limits at boot — they are applied dynamically after homing
            intakeDeployMotorConfig.softLimit
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimitEnabled(false);
            // Absolute encoder (through-bore on data port) — display only
            intakeDeployMotorConfig.absoluteEncoder
                .inverted(false);
            // Relative encoder — raw motor rotations (no conversion factor needed,
            // PID setpoints are in motor rotations from home)
            intakeDeployMotorConfig.closedLoop
                .positionWrappingEnabled(false)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD)
                .outputRange(-IntakeConstants.kMaxOutputDutyCycle, IntakeConstants.kMaxOutputDutyCycle);
        }
    }

    // Add more configs
}
