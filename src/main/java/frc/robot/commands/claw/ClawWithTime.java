package frc.robot.commands.claw;

import frc.robot.subsystems.Claw;
import harkerrobolib.commands.IndefiniteCommand;

public class ClawWithTime extends IndefiniteCommand {
    private double speed;
    public ClawWithTime(double speed) {
        this.speed = speed;
        addRequirements(Claw.getInstance());
    }

    public void execute() {
        Claw.getInstance().setPercentOutput(speed);
    }

    @Override
    public void end(boolean interrupted) {
        Claw.getInstance().setPercentOutput(0);
    }
}
