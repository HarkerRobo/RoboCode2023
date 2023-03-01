package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AlignPitch extends CommandBase {
  public static double kP = 0.3; // TODO
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double SETPOINT = 0;
  private static PIDController pitchController = new PIDController(kP, kI, kD);

  public AlignPitch() {
    addRequirements(Drivetrain.getInstance());
    pitchController.setTolerance(Drivetrain.MAX_ERROR_PITCH);
  }

  public void execute() {
    kP = SmartDashboard.getNumber("Pitch kP", kP);
    SmartDashboard.putNumber("Pitch kP", kP);
    SmartDashboard.putNumber("Pitch value", Drivetrain.getInstance().getPitch());
    double error = SETPOINT - Drivetrain.getInstance().getPitch();
    double forwardAmount = pitchController.calculate(error, SETPOINT);
    ChassisSpeeds speeds = new ChassisSpeeds(-forwardAmount, 0, 0);
    Drivetrain.getInstance().setAngleAndDrive(speeds);
  }

  @Override
  public boolean isFinished() {
    return Drivetrain.getInstance().checkPitch(SETPOINT);
  }

  public void end(boolean interrupted) {
    Drivetrain.getInstance().setAngleAndDrive(new ChassisSpeeds());
  }
}
