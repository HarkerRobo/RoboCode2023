package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;

public class AlignYaw extends CommandBase {
  public static double kP = (RobotMap.IS_COMP) ? 0.0 : 0.4; // TODO
  public static final double kI = 0.001 ;
  public static final double kD = 0.1;

  private static ProfiledPIDController thetaController =
      new ProfiledPIDController(
          kP,
          kI,
          kD,
          new Constraints(RobotMap.MAX_ANGLE_VELOCITY, RobotMap.MAX_ANGLE_ACCELERATION));

  public static final double SETPOINT = 0;

  public AlignYaw() {
    addRequirements(Drivetrain.getInstance());
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void execute() {
    kP = SmartDashboard.getNumber("Yaw kP", kP);
    SmartDashboard.putNumber("Yaw kP", kP);

    double error = SETPOINT - Drivetrain.getInstance().getHeading();
    thetaController.setP(kP);
    double rotAmt = thetaController.calculate(error);
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
