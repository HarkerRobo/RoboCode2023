package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ZeroExtend extends CommandBase{

    public ZeroExtend(){
        addRequirements(Arm.getInstance());
    }
    public void execute(){
        Arm.getInstance().setExtensionPower(-0.5);
    }

    public void end(boolean interrupted) {
        Arm.getInstance().setExtensionPower(0);
    }

    public boolean isFinished() {
        return Arm.getInstance().extensionStop();
    }
}
