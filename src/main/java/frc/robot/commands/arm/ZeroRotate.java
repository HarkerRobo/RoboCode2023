package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ZeroRotate extends CommandBase{

    public ZeroRotate(){
        addRequirements(Arm.getInstance());
    }
    public void execute(){
        while(!Arm.getInstance().rotationStop()){
            Arm.getInstance().setRotationPower(-0.5);
        }
    }

    public void end(boolean interrupted) {
    }

    public boolean isFinished() {
        return Arm.getInstance().rotationStop();
    }
    
}
