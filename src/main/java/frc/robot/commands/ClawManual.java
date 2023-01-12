package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Claw;

public class ClawManual extends InstantCommand {
    public ClawManual() {
        addRequirements(Claw.getInstance());
    }

    public void initialize() {
        Claw.getInstance().toggleClaw();
    }
}
