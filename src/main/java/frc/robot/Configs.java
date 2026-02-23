package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Current;
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
                .feedbackSensor(FeedbackSensor.kDetachedRelativeEncoder)
                .pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
            secondaryShooterConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kDetachedRelativeEncoder)
                .pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
            
        }
    }

    public static final class HoodConfigs {
        public static final SparkMaxConfig primaryHoodConfig = new SparkMaxConfig();
        public static final SparkMaxConfig secondaryHoodConfig = new SparkMaxConfig();

                static {
            primaryHoodConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(CurrentConstants.AMP20, CurrentConstants.AMP15);
            secondaryHoodConfig
                .idleMode(IdleMode.kCoast)
                .follow(CanIdConstants.kHoodPrimaryCanId, true)
                .smartCurrentLimit(CurrentConstants.AMP20, CurrentConstants.AMP15);
            primaryHoodConfig.absoluteEncoder
                .positionConversionFactor(HoodConstants.kMotorGearRatio)
                .velocityConversionFactor(HoodConstants.kMotorGearRatio);
            primaryHoodConfig.closedLoop
                .positionWrappingEnabled(true)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD)
                .outputRange(-0.7, 0.7); // Old was +-0.8
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
                .smartCurrentLimit(CurrentConstants.AMP60, CurrentConstants.AMP40);
        }
    }

    public static final class KickerConfigs {
        public static final SparkFlexConfig kickerMotorConfig = new SparkFlexConfig();
        static {
            kickerMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(CurrentConstants.AMP60, CurrentConstants.AMP40);
        }
    }

    public static final class IntakeConfigs {
        public static final SparkFlexConfig intakeMotorConfig = new SparkFlexConfig();
        public static final SparkMaxConfig IntakeDeployMotorConfig = new SparkMaxConfig();        
        
        static {
            intakeMotorConfig
                .inverted(true)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(CurrentConstants.AMP30, CurrentConstants.AMP20);
            IntakeDeployMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(CurrentConstants.AMP60, CurrentConstants.AMP40);
            IntakeDeployMotorConfig.absoluteEncoder
                .positionConversionFactor(IntakeConstants.kDeployGearRatio)
                .velocityConversionFactor(IntakeConstants.kDeployGearRatio);
            IntakeDeployMotorConfig.closedLoop
                .positionWrappingEnabled(true)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD)
                .outputRange(-0.5, 0.5); // Old was +-0.8
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
                .smartCurrentLimit(CurrentConstants.AMP40, CurrentConstants.AMP30);
            climberSecondaryMotor
                .idleMode(IdleMode.kBrake)
                .follow(CanIdConstants.kClimberPrimaryCanId, false)
                .smartCurrentLimit(CurrentConstants.AMP40, CurrentConstants.AMP30);
        }
    }

    // Add more configs
}
