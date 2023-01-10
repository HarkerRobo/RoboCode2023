package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public final class RobotMap {
    public static final Field2d FIELD = new Field2d();

    public static final String CAN_CHAIN = "BINGCHILLING";

    // Conversion Rates
    public static final double TICKS_TO_DEGRESS = 360.0 / 2048.0;
    public static final double INCHES_TO_METERS = 0.0254;
    public static final double TICKS_TO_METERS = (10.0 * Math.PI * INCHES_TO_METERS) / 2048.0;

    // Voltage Compensation
    public static final double MAX_CONTROL_EFFORT = 10.0;

    // Robot Constants
    public static final double MAX_DRIVING_SPEED = 2; // m/s
    public static final double MAX_TURNING_SPEED = 1; // rad/s

    // Robot Dimensions
    public static final double ROBOT_LENGTH = 0.0;
    public static final double ROBOT_WIDTH = 0.0;
    public static final double WHEEL_DIAMETER = 0.0;


    public static final class OI {
        public static final double DEADBAND = 0.0;

        public static final int DRIVER_ID = 0;
        public static final int OPERATOR_ID = 1;
    }
    
    public static final class SwerveModule {
        // IDs and Inverts
        public static final int[] TRANSLATION_ID = {
            0, 0, 0, 0
        };

        public static final boolean[] TRANSLATION_INVERT = {
            false, false, false, false
        };

        public static final int[] ROTATION_ID = {
            0, 0, 0, 0
        };

        public static final boolean[] ROTATION_INVERT = {
            false, false, false, false
        };

        public static final int[] CAN_CODER_ID = {
            0, 0, 0, 0
        };

        // Rolling Average
        public static final int VELOCITY_FILTER = 32;

        // Current Limiting Constants
        public static final int TRANS_PEAK_CURRENT = 0;
        public static final int TRANS_PEAK_TIME = 0;
        public static final int TRANS_CONT_CURRENT = 0;

        public static final int ROTATE_PEAK_CURRENT = 0;
        public static final int ROTATE_PEAK_TIME = 0;
        public static final int ROTATE_CONT_CURRENT = 0;

        // PID Indexes
        public static final int SLOT_INDEX = 0;
        public static final int PID_INDEX = 0;

        // PID Constants
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

    public static final class Drivetrain {
        // Pigeon ID
        public static final int PIGEON_ID = 0;

        // Translation Motor Conversions
        public static final double TRANSLATION_GEAR_RATIO = 6.75;
        public static final double TRANSLATION_CONVERSION = TICKS_TO_METERS * WHEEL_DIAMETER / TRANSLATION_GEAR_RATIO;

        // Rotation Motor Conversions
        public static final double ROTATION_GEAR_RATIO = 12.8;
        public static final double ROTATION_CONVERSION = TICKS_TO_DEGRESS / ROTATION_GEAR_RATIO;
    }
}
