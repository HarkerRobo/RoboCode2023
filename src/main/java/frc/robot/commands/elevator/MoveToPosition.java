package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AngledElevator;

public class MoveToPosition extends CommandBase {

  private double position;

  public void MoveTo(double position) {
    this.position = position;
    addRequirements(AngledElevator.getInstance());
  }

  public void execute() {
    AngledElevator.getInstance().moveToPosition(position);
  }

  public void end(boolean interrupted) {
    AngledElevator.getInstance().setExtensionPower(0);
  }

  public boolean isFinished() {
    return AngledElevator.getInstance().checkExtend(position);
  }
}
