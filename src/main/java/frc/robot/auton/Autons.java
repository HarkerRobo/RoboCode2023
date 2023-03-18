package frc.robot.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotMap;
import frc.robot.commands.claw.CloseClaw;
import frc.robot.commands.claw.OpenClaw;
import frc.robot.commands.drivetrain.AlignPitch;
import frc.robot.commands.elevator.MoveToPosition;
import frc.robot.commands.elevator.ZeroElevator;

public final class Autons {
  public static final SequentialCommandGroup topPath =
      new SequentialCommandGroup(
          new ZeroElevator(),
          new CloseClaw(),
          new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]),
          new OpenClaw(),
          new MoveToPosition(0),
          new SwervePositionController(
              Trajectories.topPath,
              () -> Rotation2d.fromDegrees(180),
              () -> Rotation2d.fromDegrees(180)));
  public static final SequentialCommandGroup middlePath =
      new SequentialCommandGroup(
          new ZeroElevator(),
          new CloseClaw(),
          new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]),
          new OpenClaw(),
          new MoveToPosition(0),
          new SwervePositionController(
              Trajectories.chargePad,
              () -> Rotation2d.fromDegrees(180),
              () -> Rotation2d.fromDegrees(180)),
          new AlignPitch());

public static final SequentialCommandGroup noAuton =
          new SequentialCommandGroup(
              new ZeroElevator(),
              new CloseClaw(),
              new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]),
              new OpenClaw(),
              new MoveToPosition(0)
          );
  public static final SequentialCommandGroup middleAndCross =
      new SequentialCommandGroup(
          new ZeroElevator(),
          new CloseClaw(),
          new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]),
          new OpenClaw(),
          new MoveToPosition(0)
              .alongWith(
                  new SwervePositionController(
                      Trajectories.middleAndCross1,
                      () -> Rotation2d.fromDegrees(180),
                      () -> Rotation2d.fromDegrees(180))),
          new SwervePositionController(
              Trajectories.middleAndCross2,
              () -> Rotation2d.fromDegrees(180),
              () -> Rotation2d.fromDegrees(180)),
          new AlignPitch());

  //   public static final SequentialCommandGroup topPathAndPush =
  //       new SequentialCommandGroup(
  //           new ZeroElevator(),
  //           new CloseClaw(),
  //           new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]),
  //           new OpenClaw(),
  //           new MoveToPosition(0),
  //           new SwervePositionController(
  //               Trajectories.topPathAndPush1,
  //               () -> Rotation2d.fromDegrees(212.64),
  //               () -> Rotation2d.fromDegrees(180)),
  //           new SwervePositionController(
  //               Trajectories.topPathAndPush2,
  //               () -> Rotation2d.fromDegrees(180),
  //               () -> Rotation2d.fromDegrees(212.64)));

  //   public static final SequentialCommandGroup bottomPathAndPush =
  //       new SequentialCommandGroup(
  //           new ZeroElevator(),
  //           new CloseClaw(),
  //           new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]),
  //           new OpenClaw(),
  //           new MoveToPosition(0),
  //           new SwervePositionController(
  //               Trajectories.bottomPathAndPush1,
  //               () -> Rotation2d.fromDegrees(180),
  //               () -> Rotation2d.fromDegrees(180)),
  //         new SwervePositionController(
  //                 Trajectories.bottomPathAndPush2,
  //                 () -> Rotation2d.fromDegrees(-133.54+360),
  //                 () -> Rotation2d.fromDegrees(180)),
  //           new SwervePositionController(
  //               Trajectories.bottomPathAndPush3,
  //               () -> Rotation2d.fromDegrees(180),
  //               () -> Rotation2d.fromDegrees(-133.54+360)));

  public static final SequentialCommandGroup bottomPath =
      new SequentialCommandGroup(
          new ZeroElevator(),
          new CloseClaw(),
          new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]),
          new OpenClaw(),
          new MoveToPosition(0),
          new SwervePositionController(
              Trajectories.bottomPath,
              () -> Rotation2d.fromDegrees(180),
              () -> Rotation2d.fromDegrees(180)));
  //   public static final SequentialCommandGroup moveForward =
  //   new SequentialCommandGroup(new SwervePositionController(Trajectories.moveForward,
  // ()->Rotation2d.fromDegrees(180), ()->Rotation2d.fromDegrees(180)));
}
