package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import harkerrobolib.util.Conversions;

public final class RobotMap {
  public static final Field2d FIELD = new Field2d();


  public static final String CAN_CHAIN = "BINGCHILLING";

  // Robot Constants
  public static final double MAX_DRIVING_SPEED = 4.0; // m/s
  public static final double MAX_TURNING_SPEED = 1.0; // rad/s
  public static final double MAX_ANGLE_VELOCITY = Math.PI * MAX_TURNING_SPEED;
  public static final double MAX_ANGLE_ACCELERATION = MAX_ANGLE_VELOCITY/2;

  // Robot Dimensions
  public static final double ROBOT_LENGTH = 0.0;
  public static final double ROBOT_WIDTH = 0.0;

  public static final boolean IS_PIGEON_UP = false;

  public static final class SwerveModule {
    // IDs and Inverts
    public static final int[] TRANSLATION_ID = {0, 0, 0, 0};

    public static final boolean[] TRANSLATION_INVERT = {false, false, false, false};

    public static final int[] ROTATION_ID = {0, 0, 0, 0};

    public static final boolean[] ROTATION_INVERT = {false, false, false, false};

    public static final int[] CAN_CODER_ID = {0, 0, 0, 0};

    public static final double[] CAN_CODER_OFFSETS = {0, 0, 0, 0};

    // Current Limiting Constants
    public static final double ROTATION_MOTOR_CURRENT_CONTINUOUS = 25;
    public static final double ROTATION_MOTOR_CURRENT_PEAK = 40;
    public static final double ROTATION_MOTOR_CURRENT_PEAK_DUR = 0.1;

    public static final double TRANS_MOTOR_CURRENT_CONTINUOUS = 30;
    public static final double TRANS_MOTOR_CURRENT_PEAK = 60;
    public static final double TRANS_MOTOR_CURRENT_PEAK_DUR = 0.1;

    // Translation Motor Conversions
    public static final double TRANSLATION_GEAR_RATIO = 6.75;
    public static final double WHEEL_DIAMETER = 4.0; // TODO: Change
    public static final double VELOCITY_CONVERSION =
        Conversions.conversionConstant(
            Conversions.System.VELOCITY, TRANSLATION_GEAR_RATIO, WHEEL_DIAMETER);

    public static final double POSITION_CONVERSION =
        Conversions.conversionConstant(
            Conversions.System.POSITION, TRANSLATION_GEAR_RATIO, WHEEL_DIAMETER);

    // Rotation Motor Conversions
    public static final double ROTATION_GEAR_RATIO = 12.8;
    // All conversion factors convert native units to wanted units
    public static final double ROTATION_CONVERSION =
        Conversions.conversionConstant(
            Conversions.System.ANGLE, ROTATION_GEAR_RATIO, WHEEL_DIAMETER);
  }
  public static final class SwervePositionController{
    public static final double X_KP = 0;
    public static final double X_KI = 0;
    public static final double X_KD = 0;
  
    public static final double Y_KP = 0;
    public static final double Y_KI = 0;
    public static final double Y_KD = 0;
  
    public static final double THETA_KP = 0;
    public static final double THETA_KI = 0.0;
    public static final double THETA_KD = 0.0;

    public static double MAX_ANGLE_VELOCITY = Math.PI;
    public static double MAX_ANGLE_ACCELERATION = Math.PI / 2;
  
  }

  public static final class Drivetrain {
    // Pigeon ID
    public static final int PIGEON_ID = 0;

    public static final double MIN_OUTPUT = 0.01;
  }

  public static final class Claw {
    public static final int CLAW_FOWARD_ID = 0;
    public static final int CLAW_REVERSE_ID = 0;
  }

  public static final class Arm {
    public static final int EXTENSION_ID = 0;
    public static final int ANGLE_ID = 0;

    public static final boolean EXTENSION_INVERTED = true;
    public static final boolean ANGLE_INVERTED = true;

    public static final double EXTENSION_CURRENT_PEAK = 0;
    public static final double EXTENSION_CURRENT_CONTINOUS = 0;
    public static final double EXTENSION_CURRENT_PEAK_DUR = 0;
    public static final double ANGLE_CURRENT_PEAK = 0;
    public static final double ANGLE_CURRENT_CONTINOUS = 0;
    public static final double ANGLE_CURRENT_PEAK_DUR = 0;

    public static final double EXTENSION_WHEEL_DIAMETER = 1.044;
    public static final double EXTENSION_GEAR_RATIO = 0;

    public static final double ANGLE_WHEEL_DIAMETER = 0;
    public static final double ANGLE_GEAR_RATIO = 0;

    public static final double CONVERSION_POSITION =
        Conversions.conversionConstant(Conversions.System.POSITION, EXTENSION_GEAR_RATIO, EXTENSION_WHEEL_DIAMETER);
    public static final double CONVERSION_ANGLE =
        Conversions.conversionConstant(Conversions.System.ANGLE, ANGLE_GEAR_RATIO, ANGLE_WHEEL_DIAMETER);

    // ground, middle, high, human player, low box, high box
    public static final double [] POSITIONS  = {0,0,0,0,0,0};

    public static final int SLOT_INDEX = 0;

  }
  
}