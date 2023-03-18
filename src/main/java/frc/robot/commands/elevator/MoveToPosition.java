package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AngledElevator;

public class MoveToPosition extends CommandBase {

  private double position;
  private final Timer timer = new Timer();

  public MoveToPosition(double position) {
    this.position = position;
    addRequirements(AngledElevator.getInstance());
  }
  public void initialize() {
    timer.reset();
    timer.start();
  }
  public void execute() {
    AngledElevator.getInstance().moveToPosition(position);
  }

  public void end(boolean interrupted) {
    AngledElevator.getInstance().setDesiredPosition(position);
    AngledElevator.getInstance().setExtensionPower(0);
  }

  public boolean isFinished() {
    return (AngledElevator.getInstance().checkExtend(position) || timer.get() > 2.5);
  }
}
