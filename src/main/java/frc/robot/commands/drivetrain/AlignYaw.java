package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.auton.Trajectories;
import frc.robot.subsystems.Drivetrain;

public class AlignYaw extends CommandBase {
  public static final double kP = 4.5; // TODO
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  private static PIDController thetaController = new PIDController(kP, kI, kD);

  public static final double MAX_ERROR_YAW = Math.toRadians(0.06);
  public static final Rotation2d SETPOINT = Rotation2d.fromDegrees(180);

  public AlignYaw() {
    addRequirements(Drivetrain.getInstance());
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    thetaController.setSetpoint(Trajectories.apply(SETPOINT).getRadians());
    thetaController.setTolerance(MAX_ERROR_YAW);
  }

  public void execute() {
    double clampAdd =
        1
            + Math.abs(
                    Trajectories.apply(SETPOINT).getRadians()
                        - Math.toRadians(Drivetrain.getInstance().getHeading()))
                * (2 / Math.PI);
    double rotAmt =
        MathUtil.clamp(
            thetaController.calculate(Math.toRadians(Drivetrain.getInstance().getHeading())),
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

  @Override
  public boolean isFinished() {
    return thetaController.atSetpoint();
  }
}
