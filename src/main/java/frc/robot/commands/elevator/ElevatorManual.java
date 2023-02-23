package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.subsystems.AngledElevator;
import harkerrobolib.commands.IndefiniteCommand;

public class ElevatorManual extends IndefiniteCommand {
  public static double PERCENT_OUTPUT = 0.3;

  public ElevatorManual() {
    addRequirements(AngledElevator.getInstance());
  }

  public void execute() {
    PERCENT_OUTPUT = SmartDashboard.getNumber("Elevator Percent Output", PERCENT_OUTPUT);
    if (OI.getInstance().getDriver().getUpDPadButton().getAsBoolean())
      AngledElevator.getInstance().setExtensionPower(PERCENT_OUTPUT);
    else if (OI.getInstance().getDriver().getDownDPadButton().getAsBoolean())
      AngledElevator.getInstance().setExtensionPower(-PERCENT_OUTPUT);
    else
      AngledElevator.getInstance().setExtensionPower(0);
  }

  @Override
  public void end(boolean interrupted) {
    AngledElevator.getInstance().setExtensionPower(0, true);
  }
}
