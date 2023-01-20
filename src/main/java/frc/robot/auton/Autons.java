package frc.robot.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotMap;
import frc.robot.commands.claw.ClawWithTime;
import frc.robot.commands.elevator.MoveToPosition;
import frc.robot.commands.elevator.ZeroElevator;

public final class Autons {
  public static final SequentialCommandGroup topPath =
      new SequentialCommandGroup(
          new ZeroElevator(),
          new ClawWithTime(RobotMap.Claw.SPEED).withTimeout(RobotMap.Claw.INTAKE_TIME),
          new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]),
          new ClawWithTime(-RobotMap.Claw.SPEED).withTimeout(RobotMap.Claw.OUTTAKE_TIME),
          new MoveToPosition(0)
              .alongWith(
                  new SwervePositionController(
                      Trajectories.topPath,
                      () -> Rotation2d.fromDegrees(0),
                      () -> Rotation2d.fromDegrees(180)))
          // , new AlignPitch()
          );
  public static final SequentialCommandGroup middlePath =
      new SequentialCommandGroup(
          new ZeroElevator(),
          new ClawWithTime(RobotMap.Claw.SPEED).withTimeout(RobotMap.Claw.INTAKE_TIME),
          new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]),
          new ClawWithTime(-RobotMap.Claw.SPEED).withTimeout(RobotMap.Claw.OUTTAKE_TIME),
          new MoveToPosition(0)
              .alongWith(
                  new SwervePositionController(
                      Trajectories.chargePad,
                      () -> Rotation2d.fromDegrees(0),
                      () -> Rotation2d.fromDegrees(180)))
          // , new AlignPitch()
          );
  public static final SequentialCommandGroup bottomPath =
      new SequentialCommandGroup(
          new ZeroElevator(),
          new ClawWithTime(RobotMap.Claw.SPEED).withTimeout(RobotMap.Claw.INTAKE_TIME),
          new MoveToPosition(RobotMap.AngledElevator.POSITIONS[2]),
          new ClawWithTime(-RobotMap.Claw.SPEED).withTimeout(RobotMap.Claw.OUTTAKE_TIME),
          new MoveToPosition(0)
              .alongWith(
                  new SwervePositionController(
                      Trajectories.bottomPath,
                      () -> Rotation2d.fromDegrees(0),
                      () -> Rotation2d.fromDegrees(180)))
          // , new AlignPitch()
          );
}
