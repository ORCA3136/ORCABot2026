package frc.robot;

public final class Constants {



    // Robot max speed
    public static final double kDriveMaxSpeedMps = 3.05; // Meters per second
    // public static final double kDriveMaxSpeedFps = 10; // Feet per second




    
    public static final class OperatorConstants {
      // Controller and button constants

      // Drive controller port numbers
      public static final int kDriverControler = 0;
      public static final int kSecondaryDriverControler = 1;

      public static final double kStickDeadband = 0.05;
      public static final double kTriggerDeadband = 0.05;
    }

    public static final class CanIdConstants {

      //NOTE: CAN ID 9 is reserved by the pigeon in SwerveSubsystem

      // Drive Vortexes
      public static final int kFrontRightDrivingCanId = 1;
      public static final int kFrontLeftDrivingCanId = 3;
      public static final int kRearRightDrivingCanId = 5;
      public static final int kRearLeftDrivingCanId = 7;

      // Drive 550s
      public static final int kFrontRightTurningCanId = 2;
      public static final int kFrontLeftTurningCanId = 4;
      public static final int kRearRightTurningCanId = 6;
      public static final int kRearLeftTurningCanId = 8;

      // Lidars
      public static final int kLidarCanId = 25;

    }



    public static final class LedColors {

        //These are all the led options, if you want more you will have to go to a REV website called "LED BLINKIN DRIVER"
    
        //Patterns:
        public static final double Rainbow_Rainbow_Pallet = -0.99;
        public static final double Rainbow_Ocean_Pallet = -0.95;
        public static final double Rainbow_Forest_Pallet = -0.91;
        public static final double Fire_Medium = -0.59;
        public static final double Fire_Large = -0.57;
    
        //Basic colors:
        public static final double Hot_Pink = 0.57;
        public static final double Dark_Red = 0.59;
        public static final double Red = 0.61;
        public static final double Red_Orange = 0.63;
        public static final double Orange = 0.65;
        public static final double Gold = 0.67;
        public static final double Yellow = 0.69;
        public static final double Lawn_Green = 0.71;
        public static final double Lime = 0.73;
        public static final double Dark_Green = 0.75;
        public static final double Green = 0.77;
        public static final double Blue_Green = 0.79;
        public static final double Aqua = 0.81;
        public static final double Sky_Blue = 0.83;
        public static final double Dark_Blue = 0.85;
        public static final double Blue = 0.87;
        public static final double Blue_Violet = 0.89;
        public static final double Violet = 0.91;
        public static final double White = 0.93;
        public static final double Gray = 0.95;
        public static final double Dark_Gray = 0.97;
        public static final double Black = 0.99;
    }

    public static final class NetworkTableNames {
        
        public static final class Odometry {
            public static final String kOdometry = "Odometry";
            public static final String kRobotPose2d = "Robot Field Position";
            public static final String kPositionX = "Position X Meters";
            public static final String kPositionY = "Position Y Meters";
            public static final String kPositionYaw = "Position Yaw Radians";
            public static final String kVelocityX = "Velocity X Meters per Second";
            public static final String kVelocityY = "Velocity Y Meters per Second";
            public static final String kVelocityYaw = "Velocity Yaw Degrees per Second";
        }
    }
}
