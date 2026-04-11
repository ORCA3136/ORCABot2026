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
                .smartCurrentLimit(CurrentConstants.AMP60, CurrentConstants.AMP80);
            secondaryShooterConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(CurrentConstants.AMP60, CurrentConstants.AMP80);
            primaryShooterConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
            secondaryShooterConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
            primaryShooterConfig.signals
                .primaryEncoderPositionPeriodMs(500)
                .primaryEncoderVelocityPeriodMs(20)
                .busVoltagePeriodMs(500)
                .outputCurrentPeriodMs(100)
                .appliedOutputPeriodMs(100);
            secondaryShooterConfig.signals
                .primaryEncoderPositionPeriodMs(500)
                .primaryEncoderVelocityPeriodMs(20)
                .busVoltagePeriodMs(500)
                .outputCurrentPeriodMs(200)
                .appliedOutputPeriodMs(200);
        }
    }

    public static final class ConveyorConfigs {
        public static final SparkFlexConfig conveyorMotorConfig = new SparkFlexConfig();
        static {
            conveyorMotorConfig
                .inverted(true)
                .idleMode(IdleMode.kCoast)
                .voltageCompensation(12)
                .smartCurrentLimit(CurrentConstants.AMP60, CurrentConstants.AMP80);
            conveyorMotorConfig.signals
                .primaryEncoderPositionPeriodMs(500)
                .primaryEncoderVelocityPeriodMs(200)
                .busVoltagePeriodMs(500)
                .outputCurrentPeriodMs(200)
                .appliedOutputPeriodMs(500);
        }
    }

    public static final class KickerConfigs {
        public static final SparkFlexConfig kickerMotorConfig = new SparkFlexConfig();
        static {
            kickerMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .voltageCompensation(12)
                .smartCurrentLimit(CurrentConstants.AMP60, CurrentConstants.AMP80);
            kickerMotorConfig.signals
                .primaryEncoderPositionPeriodMs(500)
                .primaryEncoderVelocityPeriodMs(40)
                .busVoltagePeriodMs(500)
                .outputCurrentPeriodMs(100)
                .appliedOutputPeriodMs(500);
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
                .smartCurrentLimit(CurrentConstants.AMP60, CurrentConstants.AMP80);
            intakeMotorConfig.signals
                .primaryEncoderPositionPeriodMs(500)
                .primaryEncoderVelocityPeriodMs(200)
                .busVoltagePeriodMs(500)
                .outputCurrentPeriodMs(200)
                .appliedOutputPeriodMs(500);
            intakeDeployMotorConfig
                .inverted(true)
                .idleMode(IdleMode.kCoast)
                .voltageCompensation(12)
                .smartCurrentLimit(CurrentConstants.AMP55, CurrentConstants.AMP100);
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
            intakeDeployMotorConfig.signals
                .primaryEncoderPositionPeriodMs(20)
                .primaryEncoderVelocityPeriodMs(100)
                .busVoltagePeriodMs(500)
                .outputCurrentPeriodMs(50)
                .appliedOutputPeriodMs(200)
                .absoluteEncoderPositionPeriodMs(500);
        }
    }

    // Add more configs
}
