package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AlignYaw extends CommandBase{
    private static double kP = 0;
    public AlignYaw(){
        addRequirements(Drivetrain.getInstance());
    }

    public void execute(){
        double error = 0 - Drivetrain.getInstance().getHeading();
        kP = SmartDashboard.getNumber("Yaw kP", kP);
        SmartDashboard.putNumber("Yaw kP", kP);
        double rotAmt = -kP * error;
        ChassisSpeeds chassis = new ChassisSpeeds(0, 0, rotAmt);
        Drivetrain.getInstance().setAngleAndDrive(chassis);
    }
    public void end(){
        Drivetrain.getInstance().setAngleAndDrive(new ChassisSpeeds());
    }
    public boolean isFinished(){
       return Drivetrain.getInstance().reachedYaw(0);
    }
}
