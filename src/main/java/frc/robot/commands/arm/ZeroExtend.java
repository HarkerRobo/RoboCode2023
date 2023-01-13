package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ZeroExtend extends CommandBase{

    public ZeroExtend(){
        addRequirements(Arm.getInstance());
    }
    public void execute(){
        while(!Arm.getInstance().extensionStop()){
            Arm.getInstance().setExtensionPower(-0.5);
        }
    }

    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return Arm.getInstance().extensionStop();
    }
}
