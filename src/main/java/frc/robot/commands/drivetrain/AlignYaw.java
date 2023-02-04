package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;

public class AlignYaw extends CommandBase {
  public static final double kP = 4.5; // TODO
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  private static PIDController thetaController = new PIDController(kP, kI, kD);

  public static final double SETPOINT = 180;

  public AlignYaw() {
    addRequirements(Drivetrain.getInstance());
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void execute() {
    double clampAdd =
        1
            + Math.abs(
                    SETPOINT * RobotMap.DEG_TO_RAD
                        - Drivetrain.getInstance().getHeading() * RobotMap.DEG_TO_RAD)
                * (2 / Math.PI);
    double rotAmt =
        MathUtil.clamp(
            thetaController.calculate(
                Drivetrain.getInstance().getHeading() * RobotMap.DEG_TO_RAD,
                SETPOINT * RobotMap.DEG_TO_RAD),
            -clampAdd,
            clampAdd);
    ChassisSpeeds chassis = new ChassisSpeeds(0, 0, rotAmt);
    Drivetrain.getInstance().setAngleAndDrive(chassis);
  }

  @Override
  public void end(boolean interrupted) {
    Drivetrain.getInstance().setPreviousHeading(Drivetrain.getInstance().getHeading());
    Drivetrain.getInstance().setAngleAndDrive(new ChassisSpeeds());
  }

  public boolean isFinished() {
    return Drivetrain.getInstance().checkYaw(SETPOINT);
  }
}
