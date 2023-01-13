package frc.robot.commands.arm;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class Extend extends CommandBase {

    private double position;

    public Extend(double position) {
        this.position = position;
        addRequirements(Arm.getInstance());
    }

    public void execute() {
        Arm.getInstance().extendToPosition(position);
    }

    public void end(boolean interrupted) {
    }

    public boolean isFinished() {
        return Arm.getInstance().checkExtend();
    }
}