package frc.robot.commands.elevator;

import javax.lang.model.util.ElementScanner14;

import frc.robot.OI;
import frc.robot.subsystems.AngledElevator;
import harkerrobolib.commands.IndefiniteCommand;

public class ElevatorManual extends IndefiniteCommand {

  public ElevatorManual() {
    addRequirements(AngledElevator.getInstance());
  }

  public void execute() {
    if (OI.getInstance().getDriver().getUpDPadButtonState()) {
      AngledElevator.getInstance().setDesiredPosition(AngledElevator.getInstance().getPosition() + 500);
    }
    else if (OI.getInstance().getDriver().getDownDPadButtonState()) {
      AngledElevator.getInstance().setDesiredPosition(AngledElevator.getInstance().getPosition() - 500);
    }
    AngledElevator.getInstance().moveToPosition(AngledElevator.getInstance().getDesiredPosition());
  }

  @Override
  public void end(boolean interrupted) {
    AngledElevator.getInstance().setExtensionPower(0);
  }
}