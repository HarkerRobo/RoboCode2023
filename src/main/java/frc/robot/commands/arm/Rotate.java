package frc.robot.commands.arm;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.RobotMap;
import frc.robot.OI;

public class Rotate extends CommandBase{
    
    private double angle;

    public Rotate(double angle) {
        this.angle = angle;
        addRequirements(Arm.getInstance());
    }
    
    public void initialize() {
    }
    
    public void execute() {
        Arm.getInstance().rotateToAngle(angle);
    }
    
    public void end(boolean interrupted) {
    }

    public boolean isFinished() {
        return Arm.getInstance().checkAngle(angle);
    }
}