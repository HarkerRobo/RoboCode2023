package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.commands.claw.CloseClaw;
// import frc.robot.commands.claw.OpenClaw;
import frc.robot.commands.drivetrain.AlignPitch;
import frc.robot.commands.drivetrain.AlignYaw;
// import frc.robot.commands.elevator.MoveToPosition;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.joysticks.XboxGamepad;
import harkerrobolib.util.Constants;

public class OI {
  private static OI instance;

  private CommandXboxController driver;
  private CommandXboxController operator;

  private OI() {
    driver = new CommandXboxController(Constants.DRIVER_ID);
    operator = new CommandXboxController(Constants.OPERATOR_ID);
    initBindings();
  }

  public CommandXboxController getDriver() {
    return driver;
  }

  public CommandXboxController getOperator() {
    return operator;
  }

  private void initBindings() {
    // driver
    //     .getDownDPadButton()
    //     .whilePressed(
    // new SequentialCommandGroup(
    //             new MoveToPosition(RobotMap.AngledElevator.POSITIONS[0]), new OpenClaw()));
    // driver
    //     .getRightDPadButton()
    //     .whilePressed(
    //         new SequentialCommandGroup(
    //             new MoveToPosition(RobotMap.AngledElevator.POSITIONS[1]), new OpenClaw()));
    // driver
    //     .getUpDPadButton()
    //     .whilePressed(
    //         new SequentialCommandGroup(
    //             new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]), new OpenClaw()));
    // driver
    //     .getLeftDPadButton()
    //     .whilePressed(
    //         new SequentialCommandGroup(
    //             new MoveToPosition(RobotMap.AngledElevator.POSITIONS[3]).alongWith(new
    // OpenClaw()),
    //             new CloseClaw()));
    driver.rightBumper().whileTrue(new AlignYaw());
    driver.leftBumper().whileTrue(new AlignPitch());
    // driver.getButtonStart().onTrue(new InstantCommand(() -> Drivetrain.getInstance().setYaw(0)));
    // operator
    //     .getButtonX()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               AngledElevator.getInstance().addToPositions(50);
    //             }));
  }

  public static OI getInstance() {
    if (instance == null) instance = new OI();
    return instance;
  }
}
