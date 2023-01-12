package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Claw;

public class CloseClaw extends InstantCommand {
    public CloseClaw() {
        addRequirements(Claw.getInstance());
    }

    public void initialize() {
        Claw.getInstance().release();
    }
}
