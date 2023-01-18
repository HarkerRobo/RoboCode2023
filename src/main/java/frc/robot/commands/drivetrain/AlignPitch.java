package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AlignPitch extends CommandBase {
  public static double kP = 0; //TODO

  public static final double SETPOINT = 0;

  public AlignPitch() {
    addRequirements(Drivetrain.getInstance());
  }

  public void execute() {
    kP = SmartDashboard.getNumber("Pitch kP", kP);
    SmartDashboard.putNumber("Pitch kP", kP);

    double error = SETPOINT - Drivetrain.getInstance().getPitch();
    double forwardAmount = -kP * error;
    ChassisSpeeds speeds = new ChassisSpeeds(forwardAmount, 0, 0);
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
