package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .follow(Constants.CanIdConstants.kShooterPrimaryCanId);
            kickerMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);
            primaryHoodConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);
            secondaryHoodConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .follow(Constants.CanIdConstants.kShooterPrimaryCanId);
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
                .follow(Constants.CanIdConstants.kShooterPrimaryCanId);
        }
    }

    // Add more configs
}
