package frc.robot.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.claw.CloseClaw;
// import frc.robot.commands.claw.OpenClaw;
// import frc.robot.commands.elevator.MoveToPosition;
// import frc.robot.commands.elevator.ZeroElevator;

public final class Autons {
  public static final SequentialCommandGroup topPath =
      new SequentialCommandGroup(
          //           new ZeroElevator(),
          //           new CloseClaw(),
          //           new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]),
          //           new OpenClaw(),
          //           new MoveToPosition(0)
          //               .alongWith(
          new SwervePositionController(
              Trajectories.topPath,
              () -> Rotation2d.fromDegrees(180),
              () -> Rotation2d.fromDegrees(180))); // );
  public static final SequentialCommandGroup middlePath =
      new SequentialCommandGroup(
          //   new ZeroElevator(),
          //   new CloseClaw(),
          //   new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]),
          //   new OpenClaw(),
          //   new MoveToPosition(0)
          //       .alongWith(
          new SwervePositionController(
              Trajectories.chargePad,
              () -> Rotation2d.fromDegrees(180),
              () -> Rotation2d.fromDegrees(180)));
  //   new AlignPitch());
  //   public static final SwervePositionController moveBack =
  //       new SwervePositionController(
  //           Trajectories.moveBack,
  //           () -> Rotation2d.fromDegrees(180),
  //           () -> Rotation2d.fromDegrees(180));

  //   public static final SwervePositionController moveLeft =
  //       new SwervePositionController(
  //           Trajectories.moveLeft,
  //           () -> Rotation2d.fromDegrees(180),
  //           () -> Rotation2d.fromDegrees(180));
  public static final SequentialCommandGroup topPathAndPush =
    new SequentialCommandGroup(
      //           new ZeroElevator(),
      //           new CloseClaw(),
      //           new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]),
      //           new OpenClaw(),
      //           new MoveToPosition(0)
      //               .alongWith(
        new SwervePositionController(
            Trajectories.topPathAndPush1,
            () -> Rotation2d.fromDegrees(215),
            () -> Rotation2d.fromDegrees(180)),
        new SwervePositionController(
                Trajectories.topPathAndPush2,
                () -> Rotation2d.fromDegrees(180),
                () -> Rotation2d.fromDegrees(215)));
    public static final SequentialCommandGroup bottomPathAndPush =
        new SequentialCommandGroup(
            //           new ZeroElevator(),
            //           new CloseClaw(),
            //           new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]),
            //           new OpenClaw(),
            //           new MoveToPosition(0)
            //               .alongWith(
            new SwervePositionController(
                Trajectories.bottomPathAndPush1,
                () -> Rotation2d.fromDegrees(219),
                () -> Rotation2d.fromDegrees(180)),
            new SwervePositionController(
                    Trajectories.bottomPathAndPush2,
                    () -> Rotation2d.fromDegrees(180),
                    () -> Rotation2d.fromDegrees(219)));

  public static final SequentialCommandGroup bottomPath =
      new SequentialCommandGroup(
          //           new ZeroElevator(),
          //           new CloseClaw(),
          //           new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]),
          //           new OpenClaw(),
          //           new MoveToPosition(0)
          //               .alongWith(
          new SwervePositionController(
              Trajectories.bottomPath,
              () -> Rotation2d.fromDegrees(180),
              () -> Rotation2d.fromDegrees(180)));
}
