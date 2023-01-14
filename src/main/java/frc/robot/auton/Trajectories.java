package frc.robot.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import java.util.ArrayList;
import java.util.List;

public final class Trajectories {
  public static TrajectoryConfig defaultConfig =
      new TrajectoryConfig(RobotMap.MAX_DRIVING_SPEED, RobotMap.MAX_DRIVING_SPEED / 2)
          .setKinematics(Drivetrain.getInstance().getKinematics());

  public static Trajectory chargePad =
      generateTrajectory(
          List.of(
              new Pose2d(1.81, 3.27, Rotation2d.fromDegrees(180)), // fix coords
              new Pose2d(3.87, 2.57, Rotation2d.fromDegrees(180))),
          2.0,
          1.0,
          0.0,
          0.0);

  public static Trajectory topPath =
      generateTrajectory(
          List.of(
              new Pose2d(1.81, 4.95, Rotation2d.fromDegrees(180)),
              new Pose2d(4.65, 4.71, Rotation2d.fromDegrees(180)),
              new Pose2d(7.42, 4.62, Rotation2d.fromDegrees(0))),
          2,
          1,
          0,
          0);

  public static Trajectory bottomPath =
      generateTrajectory(
          List.of(
              new Pose2d(1.81, 0.42, Rotation2d.fromDegrees(180)),
              new Pose2d(6.79, 0.94, Rotation2d.fromDegrees(180)),
              new Pose2d(7.57, 4.62, Rotation2d.fromDegrees(0))),
          2.0,
          1.0,
          0.0,
          0.0);

  public static Trajectory generateTrajectory(
      List<Pose2d> points,
      double maxVel,
      double maxAccel,
      double startVel,
      double endVel,
      TrajectoryConstraint... constraints) {
    TrajectoryConfig config = new TrajectoryConfig(maxVel, maxAccel);
    for (TrajectoryConstraint c : constraints) {
      config.addConstraint(c);
    }
    config.setStartVelocity(startVel);
    config.setEndVelocity(endVel);
    List<Translation2d> interiorPoints = new ArrayList<Translation2d>();
    for (int i = 1; i < points.size() - 1; i++) {
      interiorPoints.add(points.get(i).getTranslation());
    }
    return TrajectoryGenerator.generateTrajectory(
        points.get(0), interiorPoints, points.get(points.size() - 1), config);
  }

  /** Flips a translation to the correct side of the field based on the current alliance color. */
  public static Translation2d apply(Translation2d translation) {
    if (shouldFlip()) {
      return new Translation2d(RobotMap.Field.fieldLength - translation.getX(), translation.getY());
    } else {
      return translation;
    }
  }

  /** Flips a rotation based on the current alliance color. */
  public static Rotation2d apply(Rotation2d rotation) {
    if (shouldFlip()) {
      return new Rotation2d(-rotation.getCos(), rotation.getSin());
    } else {
      return rotation;
    }
  }

  /** Flips a pose to the correct side of the field based on the current alliance color. */
  public static Pose2d apply(Pose2d pose) {
    if (shouldFlip()) {
      return new Pose2d(
          RobotMap.Field.fieldLength - pose.getX(),
          pose.getY(),
          new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
    } else {
      return pose;
    }
  }

  /**
   * Flips a trajectory state to the correct side of the field based on the current alliance color.
   */
  public static Trajectory.State apply(Trajectory.State state) {
    if (shouldFlip()) {
      return new Trajectory.State(
          state.timeSeconds,
          state.velocityMetersPerSecond,
          state.accelerationMetersPerSecondSq,
          new Pose2d(
              RobotMap.Field.fieldLength - state.poseMeters.getX(),
              state.poseMeters.getY(),
              new Rotation2d(
                  -state.poseMeters.getRotation().getCos(),
                  state.poseMeters.getRotation().getSin())),
          -state.curvatureRadPerMeter);
    } else {
      return state;
    }
  }

  private static boolean shouldFlip() {
    return DriverStation.getAlliance() == Alliance.Red;
  }
}
