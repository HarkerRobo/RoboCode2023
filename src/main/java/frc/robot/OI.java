package frc.robot;

// import frc.robot.commands.claw.CloseClaw;
// import frc.robot.commands.claw.OpenClaw;
import frc.robot.commands.drivetrain.AlignPitch;
import frc.robot.commands.drivetrain.AlignYaw;
// import frc.robot.commands.elevator.MoveToPosition;
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
    // driver
    //     .getDownDPadButton()
    //     .whileTrue(
    // new SequentialCommandGroup(
    //             new MoveToPosition(RobotMap.AngledElevator.POSITIONS[0]), new OpenClaw()));
    // driver
    //     .getRightDPadButton()
    //     .whileTrue(
    //         new SequentialCommandGroup(
    //             new MoveToPosition(RobotMap.AngledElevator.POSITIONS[1]), new OpenClaw()));
    // driver
    //     .getUpDPadButton()
    //     .whileTrue(
    //         new SequentialCommandGroup(
    //             new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]), new OpenClaw()));
    // driver
    //     .getLeftDPadButton()
    //     .whileTrue(
    //         new SequentialCommandGroup(
    //             new MoveToPosition(RobotMap.AngledElevator.POSITIONS[3]).alongWith(new
    // OpenClaw()),
    //             new CloseClaw()));
    driver.getRightBumper().whileTrue(new AlignYaw());
    driver.getLeftBumper().whileTrue(new AlignPitch());
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
