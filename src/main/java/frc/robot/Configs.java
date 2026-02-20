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

        public static final SparkMaxConfig primaryHoodConfig = new SparkMaxConfig();
        public static final SparkMaxConfig secondaryHoodConfig = new SparkMaxConfig();
        static {
            primaryShooterConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(CurrentConstants.AMP60, CurrentConstants.AMP40);
            secondaryShooterConfig // making the secondary shooter follow the primary uninverts it, and that makes them fight each other
                .inverted(true)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(CurrentConstants.AMP60, CurrentConstants.AMP40);
            primaryShooterConfig.closedLoop
                .pid(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
            primaryHoodConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(CurrentConstants.AMP20, CurrentConstants.AMP15);
            secondaryHoodConfig
                .idleMode(IdleMode.kCoast)
                .follow(CanIdConstants.kHoodPrimaryCanId, true)
                .smartCurrentLimit(CurrentConstants.AMP20, CurrentConstants.AMP15);
            primaryHoodConfig.absoluteEncoder
                .zeroOffset(.125)
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
        public static final SparkFlexConfig kickerMotorConfig = new SparkFlexConfig();
        static {
            conveyorMotorConfig
                .inverted(true)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(CurrentConstants.AMP60, CurrentConstants.AMP40);
            kickerMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(CurrentConstants.AMP60, CurrentConstants.AMP40);
        }
    }

    public static final class IntakeConfigs {
        public static final SparkFlexConfig intakeMotorConfig = new SparkFlexConfig();
        public static final SparkFlexConfig IntakeDeployMotorConfig = new SparkFlexConfig();        
        
        static {
            intakeMotorConfig
                .inverted(true)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(CurrentConstants.AMP30, CurrentConstants.AMP20);
            IntakeDeployMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(CurrentConstants.AMP20, CurrentConstants.AMP15);
            IntakeDeployMotorConfig.closedLoop
                .positionWrappingEnabled(true)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD)
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
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(CurrentConstants.AMP20, CurrentConstants.AMP15);
            climberSecondaryMotor
                .idleMode(IdleMode.kCoast)
                .follow(CanIdConstants.kClimberPrimaryCanId, true)
                .smartCurrentLimit(CurrentConstants.AMP20, CurrentConstants.AMP15);
        }
    }

    // Add more configs
}
