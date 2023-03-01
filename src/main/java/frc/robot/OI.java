package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.AngledElevator;
import frc.robot.commands.claw.CloseClaw;
import frc.robot.commands.claw.OpenClaw;
import frc.robot.commands.drivetrain.AlignPitch;
import frc.robot.commands.drivetrain.AlignYaw;
import frc.robot.commands.elevator.MoveToPosition;
import frc.robot.commands.elevator.ZeroElevator;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.joysticks.XboxGamepad;
import harkerrobolib.util.Constants;

public class OI {
  private static OI instance;

  private XboxGamepad driver;
  private XboxGamepad operator;

  private OI() {
    driver = new XboxGamepad(Constants.DRIVER_ID);
    operator = new XboxGamepad(Constants.OPERATOR_ID);
    initBindings();
  }

  public XboxGamepad getDriver() {
    return driver;
  }

  public XboxGamepad getOperator() {
    return operator;
  }

  private void initBindings() {
    driver.getRightDPadButton().onTrue(new OpenClaw());
    driver.getLeftDPadButton().onTrue(new CloseClaw());
    driver.getButtonY().whileTrue(new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]));
    driver.getButtonX().whileTrue(new MoveToPosition(RobotMap.AngledElevator.POSITIONS[3]));
    driver.getButtonA().whileTrue(new MoveToPosition(RobotMap.AngledElevator.POSITIONS[0]));
    driver.getButtonB().whileTrue(new MoveToPosition(RobotMap.AngledElevator.POSITIONS[1]));
    driver.getRightBumper().whileTrue(new AlignYaw());
    driver.getLeftBumper().whileTrue(new AlignPitch());
    driver.getButtonStart().onTrue(new InstantCommand(() -> Drivetrain.getInstance().setYaw(0)));
    driver.getButtonSelect().onTrue(new ZeroElevator());

    operator
        .getButtonX()
        .onTrue(
            new InstantCommand(
                () -> {
                  AngledElevator.getInstance().addToPositions(50);
                }));
  }

  public static OI getInstance() {
    if (instance == null) instance = new OI();
    return instance;
  }
}
