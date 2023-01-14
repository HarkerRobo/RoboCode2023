package frc.robot.commands.elevator;

import frc.robot.OI;
import frc.robot.subsystems.AngledElevator;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.Constants;
import harkerrobolib.util.MathUtil;

public class ElevatorManual extends IndefiniteCommand {
  public ElevatorManual() {
    addRequirements(AngledElevator.getInstance());
  }

  public void execute() {
    AngledElevator.getInstance()
        .setExtensionPower(
            MathUtil.mapJoystickOutput(
                OI.getInstance().getOperator().getRightTrigger(), Constants.TRIGGER_DEADBAND));
  }

  @Override
  public void end(boolean interrupted) {
    AngledElevator.getInstance().setExtensionPower(0);
  }
}
