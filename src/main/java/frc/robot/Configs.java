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

        public static final SparkMaxConfig primaryHoodConfig = new SparkMaxConfig();
        public static final SparkMaxConfig secondaryHoodConfig = new SparkMaxConfig();
        static {
            primaryShooterConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(30);
            secondaryShooterConfig // making the secondary shooter follow the primary uninverts it, and that makes them fight each other
                .inverted(true)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(30);
            primaryHoodConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(30);
            secondaryHoodConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .follow(CanIdConstants.kHoodPrimaryCanId)
                .smartCurrentLimit(30);
            primaryHoodConfig.absoluteEncoder
                .positionConversionFactor(HoodConstants.kMotorGearRatio)
                .velocityConversionFactor(HoodConstants.kMotorGearRatio);
            primaryHoodConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD)
                .outputRange(-1, 1); // Old was +-0.8
            // primaryHoodConfig.closedLoop.feedForward
            //     .kG(HoodConstants.kG);
        }
    }

    public static final class ConveyorConfigs {
        public static final SparkFlexConfig conveyorMotorConfig = new SparkFlexConfig();
        public static final SparkFlexConfig kickerMotorConfig = new SparkFlexConfig();
        static {
            conveyorMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(30);
            kickerMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(30);
        }
    }

    public static final class IntakeConfigs {
        public static final SparkFlexConfig intakeMotorConfig = new SparkFlexConfig();
        public static final SparkFlexConfig primaryIntakeDeployMotorConfig = new SparkFlexConfig();
        public static final SparkFlexConfig secondaryIntakeDeployMotorConfig = new SparkFlexConfig();
        static {
            intakeMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(30);
            primaryIntakeDeployMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(30);
            secondaryIntakeDeployMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .follow(CanIdConstants.kIntakeDeployPrimaryCanId)
                .smartCurrentLimit(30);
        }
    }

    // Add more configs
}
