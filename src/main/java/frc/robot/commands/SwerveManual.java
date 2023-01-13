package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.Constants;
import harkerrobolib.util.MathUtil;

public class SwerveManual extends IndefiniteCommand {
  public SwerveManual() {
    addRequirements(Drivetrain.getInstance());
  }

  public void execute() {
    double vx =
        MathUtil.mapJoystickOutput(
            OI.getInstance().getDriver().getLeftX(), Constants.JOYSTICK_DEADBAND);
    double vy =
        MathUtil.mapJoystickOutput(
            OI.getInstance().getDriver().getLeftY(), Constants.JOYSTICK_DEADBAND);
    double omega =
        MathUtil.mapJoystickOutput(
            OI.getInstance().getDriver().getRightX(), Constants.JOYSTICK_DEADBAND);

    // Scaling Values
    vx = scaleValues(vx, RobotMap.MAX_DRIVING_SPEED);
    vy = scaleValues(vy, RobotMap.MAX_DRIVING_SPEED);
    omega = scaleValues(omega, RobotMap.MAX_TURNING_SPEED);

    // pigeon alignment
    omega = Drivetrain.getInstance().adjustPigeon(omega);

    if (isRobotStill(vx, vy)) {
      vx = 0;
      vy = 0;
    }

    Drivetrain.getInstance()
        .setAngleAndDrive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                vy, -vx, -omega, Drivetrain.getInstance().getRotation()));
  }

  private double scaleValues(double value, double scaleFactor) {
    return value * scaleFactor;
  }

  private boolean isRobotStill(double x, double y) {
    return Math.sqrt(x * x + y * y) < RobotMap.Drivetrain.MIN_OUTPUT;
  }

  public void end(boolean interrupted) {
    Drivetrain.getInstance().setAngleAndDrive(new ChassisSpeeds());
  }
}