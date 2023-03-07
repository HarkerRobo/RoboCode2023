package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Claw;

public class ToggleClaw extends InstantCommand {
    public ToggleClaw() {
        addRequirements(Claw.getInstance());
    }

    public void initialize() {
        Claw.getInstance().toggle();
    }
    
}
