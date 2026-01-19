package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.*;

public class Configs {
    
    public static final class ShooterConfigs {
        public static final SparkFlexConfig primaryShooterConfig = new SparkFlexConfig();
        public static final SparkFlexConfig secondaryShooterConfig = new SparkFlexConfig();

        public static final SparkMaxConfig primaryHoodConfig = new SparkMaxConfig();
        public static final SparkMaxConfig secondaryHoodConfig = new SparkMaxConfig();

        public static final SparkFlexConfig kickerMotorConfig = new SparkFlexConfig();
        static {
            primaryShooterConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast);
            secondaryShooterConfig
                .inverted(true)
                .idleMode(IdleMode.kCoast)
                .follow(CanIdConstants.kShooterPrimaryCanId);
            kickerMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);
            primaryHoodConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);
            secondaryHoodConfig
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .follow(CanIdConstants.kShooterPrimaryCanId);

            primaryHoodConfig.encoder
                .positionConversionFactor(1 / HoodConstants.kGearRatio)
                .velocityConversionFactor(1 / HoodConstants.kGearRatio);
            primaryHoodConfig.closedLoop
                .pid(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD);
            primaryHoodConfig.closedLoop.feedForward
                .kG(HoodConstants.kG);
        }
    }

    public static final class ConveyorConfigs {
        public static final SparkFlexConfig conveyorMotorConfig = new SparkFlexConfig();
        static {
            conveyorMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast);
        }
    }

    public static final class IntakeConfigs {
        public static final SparkFlexConfig intakeMotorConfig = new SparkFlexConfig();
        public static final SparkFlexConfig primaryIntakeDeploymentMotorConfig = new SparkFlexConfig();
        public static final SparkFlexConfig secondaryIntakeDeploymentMotorConfig = new SparkFlexConfig();
        static {
            intakeMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast);
            primaryIntakeDeploymentMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);
            secondaryIntakeDeploymentMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .follow(CanIdConstants.kShooterPrimaryCanId);
        }
    }

    // Add more configs
}
