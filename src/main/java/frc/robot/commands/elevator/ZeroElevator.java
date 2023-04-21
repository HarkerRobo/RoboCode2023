package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AngledElevator;

public class ZeroElevator extends CommandBase {

  public static final double ZERO_SPEED = -0.45;

  public ZeroElevator() {
    addRequirements(AngledElevator.getInstance());
  }

  public void execute() {
    AngledElevator.getInstance().setExtensionPower(ZERO_SPEED);
  }

  public void end(boolean interrupted) {
    AngledElevator.getInstance().resetEncoders();
    AngledElevator.getInstance().setDesiredPosition(0);
  }

  @Override
  public boolean isFinished() {
    return AngledElevator.getInstance().extensionStop();
  }
}
