package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.claw.ClawManual;
import frc.robot.commands.claw.ClawWithTime;
import frc.robot.commands.drivetrain.AlignPitch;
import frc.robot.commands.drivetrain.AlignYaw;
import frc.robot.commands.elevator.MoveToPosition;
import frc.robot.subsystems.AngledElevator;
import frc.robot.subsystems.Claw;
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
    driver
        .getDownDPadButton()
        .whilePressed(
            new SequentialCommandGroup(
                new MoveToPosition(RobotMap.AngledElevator.POSITIONS[0]), new ClawWithTime(-RobotMap.Claw.SPEED).withTimeout(RobotMap.Claw.OUTTAKE_TIME)));
    driver
        .getRightDPadButton()
        .whilePressed(
            new SequentialCommandGroup(
                new MoveToPosition(RobotMap.AngledElevator.POSITIONS[1]), new ClawWithTime(-RobotMap.Claw.SPEED).withTimeout(RobotMap.Claw.OUTTAKE_TIME)));
    driver
        .getUpDPadButton()
        .whilePressed(
            new SequentialCommandGroup(
                new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]), new ClawWithTime(-RobotMap.Claw.SPEED).withTimeout(RobotMap.Claw.OUTTAKE_TIME)));
    driver
        .getLeftDPadButton()
        .whilePressed(
            new SequentialCommandGroup(
                new MoveToPosition(RobotMap.AngledElevator.POSITIONS[3]), new ClawWithTime(RobotMap.Claw.SPEED).withTimeout(RobotMap.Claw.INTAKE_TIME)));
    driver.getButtonTriggerLeft().whilePressed(new AlignYaw());
    driver.getButtonTriggerRight().whilePressed(new AlignPitch());
    operator
        .getButtonX()
        .toggleOnTrue(
            new InstantCommand(
                () -> {
                  AngledElevator.getInstance().addToPositions(5);
                }));
      
  }

  public static OI getInstance() {
    if (instance == null) instance = new OI();
    return instance;
  }
}
