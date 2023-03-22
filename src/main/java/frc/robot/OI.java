package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.claw.CloseClaw;
import frc.robot.commands.claw.ToggleClaw;
import frc.robot.commands.drivetrain.AlignPitch;
import frc.robot.commands.elevator.MoveToPosition;
import frc.robot.commands.elevator.ZeroElevator;
import frc.robot.subsystems.AngledElevator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.AngledElevator.State;
import frc.robot.util.CameraPoseEstimation;
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
    driver.getRightDPadButton().onTrue(new ToggleClaw());
    driver.getLeftDPadButton().onTrue(new CloseClaw());
    driver.getButtonY().whileTrue(new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]).alongWith(new InstantCommand(()->AngledElevator.getInstance().setDesiredState(State.HIGH))));
    driver.getButtonX().whileTrue(new MoveToPosition(RobotMap.AngledElevator.POSITIONS[3]).alongWith(new InstantCommand(()->AngledElevator.getInstance().setDesiredState(State.HP))));

    driver.getButtonA().whileTrue(new MoveToPosition(RobotMap.AngledElevator.POSITIONS[0]));
    driver.getButtonB().whileTrue(new MoveToPosition(RobotMap.AngledElevator.POSITIONS[1]).alongWith(new InstantCommand(()->AngledElevator.getInstance().setDesiredState(State.MIDDLE))));
    driver.getLeftBumper().whileTrue(new AlignPitch());
    driver
        .getButtonStart()
        .onTrue(
            new InstantCommand(
                () -> {
                  Drivetrain.getInstance().setYaw(0);
                }));
    driver.getButtonSelect().onTrue(new ZeroElevator());

    operator.getButtonSelect().onTrue(new ZeroElevator());
    operator.getLeftBumper().whileTrue(new AlignPitch());
    operator.getDownDPadButton().onTrue(new MoveToPosition(RobotMap.AngledElevator.POSITIONS[0]));
  }

  public static OI getInstance() {
    if (instance == null) instance = new OI();
    return instance;
  }
}
