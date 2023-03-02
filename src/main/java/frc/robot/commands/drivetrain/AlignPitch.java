package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AlignPitch extends CommandBase {
  public static final double kP = 0.037;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double SETPOINT = 0;
  public static final double MAX_ERROR_PITCH = 0.1;
  private static PIDController pitchController = new PIDController(kP, kI, kD);

  public AlignPitch() {
    addRequirements(Drivetrain.getInstance());
    pitchController.setTolerance(MAX_ERROR_PITCH);
  }

  public void execute() {
    pitchController.setP(SmartDashboard.getNumber("Pitch kP", kP));
    SmartDashboard.putNumber("Pitch kP", kP);
    double error = SETPOINT - Drivetrain.getInstance().getRoll();
    double forwardAmount = pitchController.calculate(error, SETPOINT);
    ChassisSpeeds speeds = new ChassisSpeeds(forwardAmount, 0, 0);
    Drivetrain.getInstance().setAngleAndDrive(speeds);
  }

  @Override
  public boolean isFinished() {
    return pitchController.atSetpoint();
  }

  public void end(boolean interrupted) {
    Drivetrain.getInstance().setAngleAndDrive(new ChassisSpeeds());
  }
}
