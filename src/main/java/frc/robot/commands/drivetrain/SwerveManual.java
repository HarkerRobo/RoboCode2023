package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.Constants;
import harkerrobolib.util.MathUtil;

public class SwerveManual extends IndefiniteCommand {

  public static final double SPEED_MULTIPLIER = 0.8;

  private double vx, vy, prevvx, prevvy, omega;

  public SwerveManual() {
    addRequirements(Drivetrain.getInstance());
    prevvx = prevvy = vx = vy = omega = 0;
  }

  public void execute() {
    prevvx = vx;
    prevvy = vy;
    vx =
        MathUtil.mapJoystickOutput(
            OI.getInstance().getDriver().getLeftY(), Constants.JOYSTICK_DEADBAND);
    vy =
        MathUtil.mapJoystickOutput(
            -OI.getInstance().getDriver().getLeftX(), Constants.JOYSTICK_DEADBAND);
    omega =
        MathUtil.mapJoystickOutput(
            OI.getInstance().getDriver().getRightX(), Constants.JOYSTICK_DEADBAND);

    // Scaling Values
    vx = scaleValues(vx, RobotMap.MAX_DRIVING_SPEED) * SPEED_MULTIPLIER;
    vy = scaleValues(vy, RobotMap.MAX_DRIVING_SPEED) * SPEED_MULTIPLIER;
    omega = scaleValues(omega, RobotMap.MAX_ANGLE_VELOCITY) * SPEED_MULTIPLIER;

    // pigeon alignment
    omega = Drivetrain.getInstance().adjustPigeon(omega);

    // limit accelerationevvx);
    vy = limitAcceleration(vy, prevvy);
    vx = limitAcceleration(vx, prevvx);

    if (isRobotStill()) {
      vx = 0;
      vy = 0;
    }
    if (Math.abs(omega) < RobotMap.Drivetrain.MIN_OUTPUT) {
      omega = 0;
    }

    Drivetrain.getInstance()
        .setAngleAndDrive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                vx, vy, -omega, Drivetrain.getInstance().getRotation()));
  }

  private double limitAcceleration(double value, double prevValue) {
    if (Math.abs(value - prevValue) / Constants.ROBOT_LOOP > RobotMap.MAX_DRIVING_ACCELERATION) {
      value =
          prevValue
              + Math.signum(value - prevValue)
                  * RobotMap.MAX_DRIVING_ACCELERATION
                  * Constants.ROBOT_LOOP;
    }
    return value;
  }

  private double scaleValues(double value, double scaleFactor) {
    return value * scaleFactor;
  }

  private boolean isRobotStill() {
    return Math.sqrt(vx * vx + vy * vy) < RobotMap.Drivetrain.MIN_OUTPUT;
  }

  public void end(boolean interrupted) {
    Drivetrain.getInstance().setAngleAndDrive(new ChassisSpeeds());
  }
}
