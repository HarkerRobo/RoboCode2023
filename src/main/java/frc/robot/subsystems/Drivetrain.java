package frc.robot.subsystems;

import com.ctre.phoenix.sensors.BasePigeon;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.CameraPoseEstimation;
import frc.robot.util.SwerveModule;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance;

  private SwerveModule[] swerveModules;

  private SwerveDriveKinematics kinematics;
  private SwerveDrivePoseEstimator poseEstimator;

  private BasePigeon pigeon;
  private double prevHeading;

  public static double PIGEON_kP = (RobotMap.IS_COMP) ? 0.007 : 0.0; // TODO

  private static double MAX_ERROR_PITCH = 0; // TODO

  private static double MAX_ERROR_YAW = 0; // TODO

  private static Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.01, 0.005, 0.01);
  private static Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.05, 0.025, 0.05);

  private Drivetrain() {
    swerveModules =
        new SwerveModule[] {
          new SwerveModule(0), new SwerveModule(1), new SwerveModule(2), new SwerveModule(3)
        };

    addChild(SwerveModule.swerveIDToName(0) + " Module", swerveModules[0]);

    pigeon = (RobotMap.IS_COMP) ? new Pigeon2(RobotMap.Drivetrain.PIGEON_ID, RobotMap.CAN_CHAIN) : new PigeonIMU(RobotMap.Drivetrain.PIGEON_ID);
    prevHeading = getHeading();

    kinematics =
        new SwerveDriveKinematics(
            new Translation2d(RobotMap.ROBOT_LENGTH / 2, RobotMap.ROBOT_WIDTH / 2),
            new Translation2d(RobotMap.ROBOT_LENGTH / 2, -RobotMap.ROBOT_WIDTH / 2),
            new Translation2d(-RobotMap.ROBOT_LENGTH / 2, RobotMap.ROBOT_WIDTH / 2),
            new Translation2d(-RobotMap.ROBOT_LENGTH / 2, -RobotMap.ROBOT_WIDTH / 2));

    Pose2d initalPoseMeters = new Pose2d();

    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            Rotation2d.fromDegrees(getHeading()),
            getModulePositions(),
            initalPoseMeters,
            stateStdDevs,
            visionStdDevs);
  }

  public double adjustPigeon(double omega) {
    PIGEON_kP = SmartDashboard.getNumber("Pigeon kP", PIGEON_kP);
    SmartDashboard.putNumber("Pigeon kP", PIGEON_kP);
    if (Math.abs(omega) <= RobotMap.Drivetrain.MIN_OUTPUT)
      omega = PIGEON_kP * (prevHeading - getHeading());
    else prevHeading = getHeading();

    return omega;
  }

  public double getHeading() {
    return (RobotMap.IS_PIGEON_UP) ? -pigeon.getYaw() : pigeon.getYaw();
  }

  public double getPitch() {
    return pigeon.getPitch();
  }

  public boolean checkPitch(double desired) {
    return Math.abs(getPitch() - desired) < MAX_ERROR_PITCH;
  }

  public boolean checkYaw(double desired) {
    return Math.abs(getHeading() - desired) < MAX_ERROR_YAW;
  }

  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public void setPose(Pose2d pose) {
    swerveModules[0].zeroTranslation();
    swerveModules[1].zeroTranslation();
    swerveModules[2].zeroTranslation();
    swerveModules[3].zeroTranslation();
    setYaw(pose.getRotation().getDegrees());
    poseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
  }

  private void setYaw(double yaw) {
    pigeon.setAccumZAngle(0);
    pigeon.setYaw((RobotMap.IS_PIGEON_UP) ? -yaw : yaw);
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      swerveModules[0].getSwerveModulePosition(),
      swerveModules[1].getSwerveModulePosition(),
      swerveModules[2].getSwerveModulePosition(),
      swerveModules[3].getSwerveModulePosition()
    };
  }

  public void updatePose() {
    poseEstimator.update(getRotation(), getModulePositions());
  }

  public void setAngleAndDrive(ChassisSpeeds chassis) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassis);
    swerveModules[0].setAngleAndDrive(states[0]);
    swerveModules[1].setAngleAndDrive(states[1]);
    swerveModules[2].setAngleAndDrive(states[2]);
    swerveModules[3].setAngleAndDrive(states[3]);
  }

  public Pose2d getPoseEstimatorPose2d() {
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    updatePose();
    Pair<Pose2d, Double> cameraPose =
        CameraPoseEstimation.getInstance()
            .getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
    poseEstimator.addVisionMeasurement(cameraPose.getFirst(), cameraPose.getSecond());
  }

  public static Drivetrain getInstance() {
    if (instance == null) instance = new Drivetrain();
    return instance;
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }
}
