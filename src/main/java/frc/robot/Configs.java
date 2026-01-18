package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Configs {
    
    public static final class ShooterConfigs {
        // Initialize individual sparkmax configs
        public static final SparkFlexConfig primaryMotorConfig = new SparkFlexConfig();
        public static final SparkFlexConfig secondaryMotorConfig = new SparkFlexConfig();

        static {
            // Add the values
            primaryMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast);
            
            secondaryMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .follow(Constants.CanIdConstants.kShooterPrimaryCanId);
        }
    }

    // Add more configs
}
