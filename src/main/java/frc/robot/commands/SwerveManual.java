package frc.robot.commands;

import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

public class SwerveManual extends IndefiniteCommand {
    public SwerveManual() {
        addRequirements(Drivetrain.getInstance());
    }

    public void initialize() {
    }   

    public void execute() {
        double transX = OI.getInstance().getDriver().getLeftX();
        double transY = OI.getInstance().getDriver().getLeftY();
        double rotate = OI.getInstance().getDriver().getRightX();

        // Eliminate Deadband
        transX = MathUtil.mapJoystickOutput(transX, RobotMap.OI.DEADBAND);
        transY = MathUtil.mapJoystickOutput(transY, RobotMap.OI.DEADBAND);
        rotate = MathUtil.mapJoystickOutput(rotate, RobotMap.OI.DEADBAND);

        // Scaling Values
        transX = scaleValues(transX, RobotMap.MAX_DRIVING_SPEED);
        transY = scaleValues(transY, RobotMap.MAX_DRIVING_SPEED);
        rotate = scaleValues(rotate, RobotMap.MAX_TURNING_SPEED);
    }

    public double scaleValues(double value, double max) {
        return value * max;
    }

    public void end(boolean interrupted) {

    }
}