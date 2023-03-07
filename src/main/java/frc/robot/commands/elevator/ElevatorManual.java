package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.subsystems.AngledElevator;
import harkerrobolib.commands.IndefiniteCommand;

public class ElevatorManual extends IndefiniteCommand {
  public static double PERCENT_OUTPUT = 0.5;

  public ElevatorManual() {
    addRequirements(AngledElevator.getInstance());
  }

  public void execute() {
    PERCENT_OUTPUT = SmartDashboard.getNumber("Elevator Percent Output", PERCENT_OUTPUT);
    if (OI.getInstance().getDriver().getUpDPadButtonState())
      AngledElevator.getInstance().setExtensionPower(PERCENT_OUTPUT);
    else if (OI.getInstance().getDriver().getDownDPadButtonState())
      AngledElevator.getInstance().setExtensionPower(-PERCENT_OUTPUT);
    else AngledElevator.getInstance().moveToPosition(AngledElevator.getInstance().getPosition());
  }

  @Override
  public void end(boolean interrupted) {
    AngledElevator.getInstance().setExtensionPower(0);
  }
}
