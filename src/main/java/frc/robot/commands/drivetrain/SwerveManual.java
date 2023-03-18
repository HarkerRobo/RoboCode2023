package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.AngledElevator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.CameraPoseEstimation;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.Constants;
import harkerrobolib.util.MathUtil;

public class SwerveManual extends IndefiniteCommand {

  public static final double SPEED_MULTIPLIER = 0.7;
  public static final double ROT_MULITPLLIER = 0.25;
  public static final double CLAMP_MULTIPLIER = 0.30;

  private static final double THETA_P = 3.95; //TODO
  private static final double THETA_I = 0.0; //TODO
  private static final double THETA_D = 0.0; //TODO
  private static ProfiledPIDController thetaController = new ProfiledPIDController(THETA_P, THETA_I, THETA_D, new Constraints(RobotMap.MAX_DRIVING_SPEED, RobotMap.MAX_DRIVING_SPEED / 2));
public static final double MAX_ERROR_YAW = Math.toRadians(0.1);
public static final double OFFSET = Math.toRadians(12); // TODO

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
    vx = scaleValues(vx, RobotMap.MAX_DRIVING_SPEED) * ((AngledElevator.getInstance().isFarExtended()) ? CLAMP_MULTIPLIER : SPEED_MULTIPLIER);
    vy = scaleValues(vy, RobotMap.MAX_DRIVING_SPEED) * ((AngledElevator.getInstance().isFarExtended()) ? CLAMP_MULTIPLIER : SPEED_MULTIPLIER);
    omega = scaleValues(omega, RobotMap.MAX_ANGLE_VELOCITY) * ((AngledElevator.getInstance().isFarExtended()) ? ROT_MULITPLLIER : SPEED_MULTIPLIER);


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

    if (OI.getInstance().getDriver().getRightBumperState()) {
      var result = CameraPoseEstimation.getInstance().getCamera().getLatestResult();
      if (result.hasTargets()) {
          omega =
              -thetaController.calculate(Math.toRadians(result.getBestTarget().getYaw()) - OFFSET);
          Drivetrain.getInstance().setPreviousHeading(Drivetrain.getInstance().getHeading());
      }
      thetaController.reset(new State());
      thetaController.setTolerance(MAX_ERROR_YAW);
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
