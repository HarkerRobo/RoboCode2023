package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Claw;

public class OpenClaw extends InstantCommand {
    public OpenClaw() {
        addRequirements(Claw.getInstance());
    }

    public void initialize() {
        Claw.getInstance().pinch();
    }
}
