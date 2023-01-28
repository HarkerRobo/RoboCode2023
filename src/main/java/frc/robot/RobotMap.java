package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import harkerrobolib.util.Conversions;

public final class RobotMap {
  public static final class Field {
    public static final double fieldLength = Units.inchesToMeters(651.25);
    public static final double fieldWidth = Units.inchesToMeters(315.5);
    public static final double tapeWidth = Units.inchesToMeters(2.0);

    public static final Field2d FIELD = new Field2d();

    public static final boolean IS_FLIPPED = false;
  }

  public static final boolean IS_COMP = false;
  public static final String CAN_CHAIN = "rio";

  // Robot Constants
  public static final double MAX_DRIVING_SPEED = 4.0; // m/s
  public static final double MAX_ANGLE_VELOCITY = Math.PI;
  public static final double MAX_ANGLE_ACCELERATION = MAX_ANGLE_VELOCITY / 2;

  // Robot Dimensions
  public static final double ROBOT_LENGTH = Units.inchesToMeters(30);
  public static final double ROBOT_WIDTH = Units.inchesToMeters(28);

  public static final class SwerveModule {
    // IDs and Inverts

    // fl, fr, bl, br
    public static final int[] TRANSLATION_ID =
        (IS_COMP) ? new int[] {0, 0, 0, 0} : new int[] {6, 3, 5, 8}; // TODO

    public static final boolean[] TRANSLATION_INVERT =
        (IS_COMP)
            ? new boolean[] {false, false, false, false}
            : new boolean[] {true, true, true, true}; // TODO

    public static final int[] ROTATION_ID =
        (IS_COMP) ? new int[] {0, 0, 0, 0} : new int[] {1, 2, 7, 4}; // TODO

    public static final boolean[] ROTATION_INVERT =
        (IS_COMP)
            ? new boolean[] {false, false, false, false}
            : new boolean[] {false, false, false, false}; // TODO

    public static final int[] CAN_CODER_ID =
        (IS_COMP) ? new int[] {0, 0, 0, 0} : new int[] {9, 10, 11, 12}; // TODO

    public static final double[] CAN_CODER_OFFSETS =
        (IS_COMP)
            ? new double[] {0, 0, 0, 0}
            : new double[] {88.242, 107.051, 66.006, 8.35}; // TODO

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
    public static final int PIGEON_ID = (IS_COMP) ? 0 : 1; // TODO

    public static final boolean IS_PIGEON_INVERTED = (IS_COMP) ? false : true; // TODO

    public static final double MIN_OUTPUT = 0.00001;
  }

  public static final class Claw {
    public static final int CLAW_FOWARD_ID = (IS_COMP) ? 0 : 0; // TODO
    public static final int CLAW_REVERSE_ID = (IS_COMP) ? 0 : 0; // TODO
  }

  public static final class AngledElevator {
    public static final int MASTER_ID = (IS_COMP) ? 0 : 0; // TODO
    public static final int FOLLOWER_ID = (IS_COMP) ? 0 : 0; // TODO

    public static final boolean MASTER_INVERTED = (IS_COMP) ? true : true; // TODO
    public static final boolean FOLLOWER_INVERTED = (IS_COMP) ? true : true; // TODO

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
