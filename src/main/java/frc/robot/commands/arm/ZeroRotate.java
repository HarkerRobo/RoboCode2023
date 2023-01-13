package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ZeroRotate extends CommandBase{

    public ZeroRotate(){
        addRequirements(Arm.getInstance());
    }
    public void execute(){
        Arm.getInstance().setRotationPower(-0.5);
    }

    public void end(boolean interrupted) {
        Arm.getInstance().setRotationPower(0);
    }

    public boolean isFinished() {
        return Arm.getInstance().rotationStop();
    }
    
}
