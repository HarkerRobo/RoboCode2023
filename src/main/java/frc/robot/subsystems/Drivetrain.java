package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.SwerveModule;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance;

  private SwerveModule[] swerveModules;

  private SwerveDriveKinematics kinematics;
  private SwerveDrivePoseEstimator poseEstimator;

  private Pigeon2 pigeon;

  private double prevHeading;

  private Drivetrain() {
    swerveModules =
        new SwerveModule[] {
          new SwerveModule(0), new SwerveModule(1), new SwerveModule(2), new SwerveModule(3)
        };

    addChild(SwerveModule.swerveIDToName(0) + " Module", swerveModules[0]);

    pigeon = new Pigeon2(RobotMap.Drivetrain.PIGEON_ID, RobotMap.CAN_CHAIN);
    prevHeading = getHeading();

    kinematics =
        new SwerveDriveKinematics(
            new Translation2d(RobotMap.ROBOT_LENGTH / 2, RobotMap.ROBOT_WIDTH / 2),
            new Translation2d(RobotMap.ROBOT_LENGTH / 2, -RobotMap.ROBOT_WIDTH / 2),
            new Translation2d(-RobotMap.ROBOT_LENGTH / 2, RobotMap.ROBOT_WIDTH / 2),
            new Translation2d(-RobotMap.ROBOT_LENGTH / 2, -RobotMap.ROBOT_WIDTH / 2));
  }

  public double getHeading() {
    return -pigeon.getYaw();
  }

  public double adjustPigeon(double omega) {
    if (Math.abs(omega) <= RobotMap.Drivetrain.MIN_OUTPUT)
      omega = -RobotMap.Drivetrain.PIGEON_kP * (getHeading() - prevHeading);
    else prevHeading = getHeading();

    return omega;
  }

  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public void updatePose() {}

  public void setAngleAndDrive(ChassisSpeeds chassis) {}

  public Pose2d getPoseEstimatorPose2d() {
    return poseEstimator.getEstimatedPosition();
  }


  @Override
  public void periodic() {
    updatePose();
  }

  public static Drivetrain getInstance() {
    if (instance == null) instance = new Drivetrain();
    return instance;
  }
}
