package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AngledElevator;

public class MoveToPosition extends CommandBase {

  private double position;
  private double time;

  public MoveToPosition(double position) {
    this.position = position;
    addRequirements(AngledElevator.getInstance());
    time = Timer.getFPGATimestamp();
  }

  public void execute() {
    AngledElevator.getInstance().moveToPosition(position);
  }

  public void end(boolean interrupted) {
    AngledElevator.getInstance().resetPosition();
  }

  public boolean isFinished() {
    return AngledElevator.getInstance().checkExtend(position)
        || Timer.getFPGATimestamp() - time > 5.5;
  }
}
