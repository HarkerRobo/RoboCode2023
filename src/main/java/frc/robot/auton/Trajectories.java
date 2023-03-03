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
              new Pose2d(1.91, 2.75, Rotation2d.fromDegrees(180)),
              new Pose2d(4.50, 2.75, Rotation2d.fromDegrees(181))),
          3.0,
          2.0,
          0.0,
          0.0, true);

  public static Trajectory topPathAndPush1 =
      generateTrajectory(
          List.of(
              new Pose2d(1.91, 4.44, Rotation2d.fromDegrees(180.0)),
              new Pose2d(4.94, 5.01, Rotation2d.fromDegrees(180.0)),
              new Pose2d(7.52, 5.58, Rotation2d.fromDegrees(212.64))),
          2.0,
          1.5,
          0,
          1.0, true);

  public static Trajectory topPathAndPush2 =
      generateTrajectory(
          List.of(
              new Pose2d(7.52, 5.58, Rotation2d.fromDegrees(212.64)),
              new Pose2d(7.79, 4.66, Rotation2d.fromDegrees(180.0)),
              new Pose2d(4.57, 4.62, Rotation2d.fromDegrees(180.0)),
              new Pose2d(1.91, 4.44, Rotation2d.fromDegrees(180.0))),
          2.0,
          1.5,
          1.0,
          0, false);

  public static Trajectory topPath =
      generateTrajectory(
          List.of(
              new Pose2d(1.91, 4.44, Rotation2d.fromDegrees(180)),
              new Pose2d(4.08, 4.80, Rotation2d.fromDegrees(180)),
              new Pose2d(7.42, 4.62, Rotation2d.fromDegrees(180))),
          2.0,
          1.0,
          0,
          0, true);

  public static Trajectory bottomPathAndPush1 =
      generateTrajectory(
          List.of(
              new Pose2d(1.91, 1.09, Rotation2d.fromDegrees(180.0)),
              new Pose2d(4.13, 0.70, Rotation2d.fromDegrees(180.0)),
              new Pose2d(7.49, 2.03, Rotation2d.fromDegrees(133.54))),
          2.0,
          1.5,
          0.0,
          1.0, true);
  public static Trajectory bottomPathAndPush2 =
      generateTrajectory(
          List.of(
              new Pose2d(7.49, 2.03, Rotation2d.fromDegrees(133.54)),
              new Pose2d(7.87, 0.91, Rotation2d.fromDegrees(180.0)),
              new Pose2d(4.14, 0.71, Rotation2d.fromDegrees(180.0)),
              new Pose2d(1.91, 1.09, Rotation2d.fromDegrees(180.0))),
          2.0,
          1.5,
          1.0,
          0.0, false);

//   public static Trajectory roomAuto = 
//         generateTrajectory(List.of(new Pose2d(1.91, 1.09, Rotationd.fromDegrees(180.0)), new Pose2d()), 0, 0, 0, 0, isFlipped(), null)
  public static Trajectory bottomPath =
      generateTrajectory(
          List.of(
              new Pose2d(1.91, 1.09, Rotation2d.fromDegrees(180)),
              new Pose2d(6.33, 0.70, Rotation2d.fromDegrees(180))),
          2.0,
          1.0,
          0.0,
          0.0,
          true);

  public static Trajectory generateTrajectory(
      List<Pose2d> points,
      double maxVel,
      double maxAccel,
      double startVel,
      double endVel,
      boolean reversed,
      TrajectoryConstraint... constraints) {
    TrajectoryConfig config = new TrajectoryConfig(maxVel, maxAccel);
    for (TrajectoryConstraint c : constraints) {
      config.addConstraint(c);
    }
    config.setStartVelocity(startVel);
    config.setEndVelocity(endVel);
    config.setReversed(reversed);
    List<Translation2d> interiorPoints = new ArrayList<Translation2d>();
    for (int i = 1; i < points.size() - 1; i++) {
      interiorPoints.add(points.get(i).getTranslation());
    }
    return TrajectoryGenerator.generateTrajectory(
        points.get(0), interiorPoints, points.get(points.size() - 1), config);
  }

  /** Flips a translation to the correct side of the field based on the current alliance color. */
  public static Translation2d apply(Translation2d translation) {
    return (isFlipped())
        ? new Translation2d(RobotMap.Field.fieldLength - translation.getX(), translation.getY())
        : translation;
  }

  /** Flips a rotation based on the current alliance color. */
  public static Rotation2d apply(Rotation2d rotation) {
    return (isFlipped()) ? new Rotation2d(-rotation.getCos(), rotation.getSin()) : rotation;
  }

  /** Flips a pose to the correct side of the field based on the current alliance color. */
  public static Pose2d apply(Pose2d pose) {
    return (isFlipped())
        ? new Pose2d(
            RobotMap.Field.fieldLength - pose.getX(),
            pose.getY(),
            new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()))
        : pose;
  }

  /**
   * Flips a trajectory state to the correct side of the field based on the current alliance color.
   */
  public static Trajectory.State apply(Trajectory.State state) {
    return (isFlipped())
        ? new Trajectory.State(
            state.timeSeconds,
            state.velocityMetersPerSecond,
            state.accelerationMetersPerSecondSq,
            new Pose2d(
                RobotMap.Field.fieldLength - state.poseMeters.getX(),
                state.poseMeters.getY(),
                new Rotation2d(
                    -state.poseMeters.getRotation().getCos(),
                    state.poseMeters.getRotation().getSin())),
            -state.curvatureRadPerMeter)
        : state;
  }

  private static boolean isFlipped() {
    return DriverStation.getAlliance() == Alliance.Red;
  }
}
