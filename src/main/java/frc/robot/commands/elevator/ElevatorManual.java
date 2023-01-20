package frc.robot.commands.elevator;

import frc.robot.OI;
import frc.robot.subsystems.AngledElevator;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.Constants;

public class ElevatorManual extends IndefiniteCommand {
  public ElevatorManual() {
    addRequirements(AngledElevator.getInstance());
  }

  public void execute() {
    double rightTrigger = OI.getInstance().getOperator().getRightTrigger();
    double leftTrigger = OI.getInstance().getOperator().getLeftTrigger();
    if (rightTrigger > Constants.TRIGGER_DEADBAND)
      AngledElevator.getInstance().setExtensionPower(rightTrigger);
    else if (leftTrigger > Constants.TRIGGER_DEADBAND)
      AngledElevator.getInstance().setExtensionPower(-leftTrigger);
    // AngledElevator.getInstance()
    //     .setExtensionPower(
    //         MathUtil.mapJoystickOutput(
    //             OI.getInstance().getOperator().getRightY(), Constants.JOYSTICK_DEADBAND));
  }

  @Override
  public void end(boolean interrupted) {
    AngledElevator.getInstance().setExtensionPower(0);
  }
}
