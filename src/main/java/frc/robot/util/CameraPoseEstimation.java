package frc.robot.util;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

public final class CameraPoseEstimation {
  private static CameraPoseEstimation instance;

  private AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonCamera cam;
  Transform3d robotToCam;
  RobotPoseEstimator robotPoseEstimator;

  public static final List<AprilTag> aprilTags =
      List.of(
          new AprilTag(
              1,
              new Pose3d(
                  Units.inchesToMeters(610.77),
                  Units.inchesToMeters(42.19),
                  Units.inchesToMeters(18.22),
                  new Rotation3d(0.0, 0.0, Math.PI))),
          new AprilTag(
              2,
              new Pose3d(
                  Units.inchesToMeters(610.77),
                  Units.inchesToMeters(108.19),
                  Units.inchesToMeters(18.22),
                  new Rotation3d(0.0, 0.0, Math.PI))),
          new AprilTag(
              3,
              new Pose3d(
                  Units.inchesToMeters(610.77),
                  Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
                  Units.inchesToMeters(18.22),
                  new Rotation3d(0.0, 0.0, Math.PI))),
          new AprilTag(
              4,
              new Pose3d(
                  Units.inchesToMeters(636.96),
                  Units.inchesToMeters(265.74),
                  Units.inchesToMeters(27.38),
                  new Rotation3d(0.0, 0.0, Math.PI))),
          new AprilTag(
              5,
              new Pose3d(
                  Units.inchesToMeters(14.25),
                  Units.inchesToMeters(265.74),
                  Units.inchesToMeters(27.38),
                  new Rotation3d())),
          new AprilTag(
              6,
              new Pose3d(
                  Units.inchesToMeters(40.45),
                  Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
                  Units.inchesToMeters(18.22),
                  new Rotation3d())),
          new AprilTag(
              7,
              new Pose3d(
                  Units.inchesToMeters(40.45),
                  Units.inchesToMeters(108.19),
                  Units.inchesToMeters(18.22),
                  new Rotation3d())),
          new AprilTag(
              8,
              new Pose3d(
                  Units.inchesToMeters(40.45),
                  Units.inchesToMeters(42.19),
                  Units.inchesToMeters(18.22),
                  new Rotation3d())));

  public CameraPoseEstimation() {
    aprilTagFieldLayout = new AprilTagFieldLayout(CameraPoseEstimation.aprilTags, 16.4846, 8.1026);
    cam = new PhotonCamera("limelight");
    robotToCam =
        new Transform3d(
            new Translation3d(0, Units.inchesToMeters(10.81259), 0), new Rotation3d(0, 0, 0));
    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<PhotonCamera, Transform3d>(cam, robotToCam));
    robotPoseEstimator =
        new RobotPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
  }

  public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    double currentTime = Timer.getFPGATimestamp();
    Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
    if (result.isPresent()) {
      return new Pair<Pose2d, Double>(
          result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
    } else {
      return new Pair<Pose2d, Double>(null, 0.0);
    }
  }

  public static CameraPoseEstimation getInstance() {
    if (instance == null) {
      instance = new CameraPoseEstimation();
    }
    return instance;
  }
}
