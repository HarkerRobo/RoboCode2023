package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AngledElevator;

public class ZeroElevator extends CommandBase{

    public ZeroElevator(){
        addRequirements(AngledElevator.getInstance());
    }
    public void execute(){
        AngledElevator.getInstance().setExtensionPower(-0.5);
    }

    public void end(boolean interrupted) {
        AngledElevator.getInstance().setExtensionPower(0);
    }

    public boolean isFinished() {
        return AngledElevator.getInstance().extensionStop();
    }
}