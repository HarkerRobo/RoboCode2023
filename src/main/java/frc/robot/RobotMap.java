package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import harkerrobolib.util.Conversions;

public final class RobotMap {
  public static final class Field {
    public static final double fieldLength = Units.inchesToMeters(651.25);
    public static final double fieldWidth = Units.inchesToMeters(315.5);
    public static final double tapeWidth = Units.inchesToMeters(2.0);
  }

  public static final Field2d FIELD = new Field2d();

  public static final String CAN_CHAIN = "BINGCHILLING";

  // Robot Constants
  public static final double MAX_DRIVING_SPEED = 4.0; // m/s
  public static final double MAX_TURNING_SPEED = 1.0; // rad/s
  public static final double MAX_ANGLE_VELOCITY = Math.PI * MAX_TURNING_SPEED;
  public static final double MAX_ANGLE_ACCELERATION = MAX_ANGLE_VELOCITY / 2;

  // Robot Dimensions
  public static final double ROBOT_LENGTH = Units.inchesToMeters(28);
  public static final double ROBOT_WIDTH = Units.inchesToMeters(30);

  public static final boolean IS_PIGEON_UP = false;

  public static final class SwerveModule {
    // IDs and Inverts
    public static final int[] TRANSLATION_ID = {0, 0, 0, 0}; // TODO

    public static final boolean[] TRANSLATION_INVERT = {false, false, false, false}; // TODO

    public static final int[] ROTATION_ID = {0, 0, 0, 0}; // TODO

    public static final boolean[] ROTATION_INVERT = {false, false, false, false}; // TODO

    public static final int[] CAN_CODER_ID = {0, 0, 0, 0}; // TODO

    public static final double[] CAN_CODER_OFFSETS = {0, 0, 0, 0}; // TODO

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

  public static final class Drivetrain {
    // Pigeon ID
    public static final int PIGEON_ID = 0; // TODO

    public static final double MIN_OUTPUT = 0.01;
  }

  public static final class Claw {
    public static final int CLAW_FOWARD_ID = 0; // TODO
    public static final int CLAW_REVERSE_ID = 0; // TODO
  }

  public static final class AngledElevator {
    public static final int MASTER_ID = 0; // TODO
    public static final int FOLLOWER_ID = 0; // TODO

    public static final boolean MASTER_INVERTED = true; // TODO
    public static final boolean FOLLOWER_INVERTED = true; // TODO

    public static final double MASTER_CURRENT_PEAK = 0; // TODO
    public static final double MASTER_CURRENT_CONTINOUS = 0; // TODO
    public static final double MASTER_CURRENT_PEAK_DUR = 0; // TODO
    public static final double FOLLOWER_CURRENT_PEAK = 0; // TODO
    public static final double FOLLOWER_CURRENT_CONTINOUS = 0; // TODO
    public static final double FOLLOWER_CURRENT_PEAK_DUR = 0; // TODO

    // low, middle, high, human player
    public static final double[] POSITIONS = {0, 0, 0, 0}; // TODO
  }
}
