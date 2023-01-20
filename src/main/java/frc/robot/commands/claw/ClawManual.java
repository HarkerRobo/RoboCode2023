package frc.robot.commands.claw;

import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.Claw;
import harkerrobolib.commands.IndefiniteCommand;

public class ClawManual extends IndefiniteCommand {
    public ClawManual() {
        addRequirements(Claw.getInstance());
    }

    public void execute() {
        if(OI.getInstance().getDriver().getRightTrigger() > 0.5) {
            Claw.getInstance().setPercentOutput(RobotMap.Claw.SPEED);
        }

        else if(OI.getInstance().getDriver().getLeftTrigger() > 0.5) {
            Claw.getInstance().setPercentOutput(-RobotMap.Claw.SPEED);
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        Claw.getInstance().setPercentOutput(0);
    }
}
