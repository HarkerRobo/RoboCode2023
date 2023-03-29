package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
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

  private Pigeon2 pigeon;
  private double prevHeading;

  public static double PIGEON_kP = 0.9;

  private static Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.01);
  private static Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.05, 0.025, 0.05);

  private static final double THETA_P = 0.118; //TODO
  private static final double THETA_I = 0.0; //TODO
  private static final double THETA_D = 0.0; //TODO
  
  private static ProfiledPIDController thetaController = new ProfiledPIDController(THETA_P, THETA_I, THETA_D, new Constraints(4, 3.5));
  public static final double MAX_ERROR_YAW = 0.5;

  public static final double OFFSET = 9.5;

  private Drivetrain() {
    swerveModules =
        new SwerveModule[] {
          new SwerveModule(0), new SwerveModule(1), new SwerveModule(2), new SwerveModule(3)
        };

    pigeon = new Pigeon2(RobotMap.Drivetrain.PIGEON_ID);
    initPigeon();

    kinematics =
        new SwerveDriveKinematics(
            new Translation2d(RobotMap.ROBOT_LENGTH / 2, RobotMap.ROBOT_WIDTH / 2),
            new Translation2d(RobotMap.ROBOT_LENGTH / 2, -RobotMap.ROBOT_WIDTH / 2),
            new Translation2d(-RobotMap.ROBOT_LENGTH / 2, RobotMap.ROBOT_WIDTH / 2),
            new Translation2d(-RobotMap.ROBOT_LENGTH / 2, -RobotMap.ROBOT_WIDTH / 2));
   thetaController.setTolerance(MAX_ERROR_YAW);

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

  private void initPigeon() {
    pigeon.configFactoryDefault();
    pigeon.configMountPoseYaw(90);
    pigeon.configMountPosePitch(0);
    pigeon.configMountPoseRoll(0);
    pigeon.setYaw(0);
    pigeon.configEnableCompass(false);
  }

  public double adjustPigeon(double omega) {
    PIGEON_kP = SmartDashboard.getNumber("Pigeon kP", PIGEON_kP);
    SmartDashboard.putNumber("Pigeon kP", PIGEON_kP);
    if (Math.abs(omega) <= RobotMap.Drivetrain.MIN_OUTPUT)
      omega = -PIGEON_kP * (prevHeading - getHeading());
    else prevHeading = getHeading();

    return omega;
  }

  public double getHeading() {
    SmartDashboard.putNumber("pigeon heading", pigeon.getYaw());
    return pigeon.getYaw();
  }

  public double getPitch() {
    return pigeon.getPitch();
  }

  public double getRoll() {
    return pigeon.getRoll();
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

  public void setYaw(double yaw) {
    pigeon.setYaw(yaw);
    setPreviousHeading(yaw);
  }

  public void setPreviousHeading(double prev) {
    prevHeading = prev;
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

  public double alignToTarget(double omega) {
    var result = CameraPoseEstimation.getInstance().getCamera().getLatestResult();
    if (result.hasTargets()) {
        omega =
            -thetaController.calculate(result.getBestTarget().getYaw() - OFFSET);
        setPreviousHeading(getHeading());
    }
    return omega;
  }

  // public double getOffset() {
  //   switch (AngledElevator.getInstance().getState()) {
  //     case MIDDLE:
  //       return 18;
  //     case HIGH:
  //       return 10.5;
  //     case HP:
  //       return 9.5;
  //     default:
  //       return 11;
  //   }
  // }

  @Override
  public void periodic() {
    updatePose();
    // Pair<Pose2d, Double> cameraPose =
    //     CameraPoseEstimation.getInstance()
    //         .getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
    // poseEstimator.addVisionMeasurement(cameraPose.getFirst(), cameraPose.getSecond());
  }

  public static Drivetrain getInstance() {
    if (instance == null) instance = new Drivetrain();
    return instance;
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Drivetrain");
    builder.setActuator(true);
    builder.setSafeState(() -> setAngleAndDrive(new ChassisSpeeds()));
    builder.addDoubleProperty(
        "Translation kS", () -> SwerveModule.TRANSLATION_kS, swerveModules[0]::setkS);
    builder.addDoubleProperty(
        "Translation kV", () -> SwerveModule.TRANSLATION_kV, swerveModules[0]::setkV);
    builder.addDoubleProperty("Pitch Value", () -> getPitch(), null);
    builder.addDoubleProperty("Roll Value", () -> getRoll(), null);

    for (int i = 0; i < 4; i++) {
      builder.addDoubleProperty(
          SwerveModule.swerveIDToName(i) + " Translation Speed", swerveModules[i]::getSpeed, null);
      builder.addDoubleProperty(
          SwerveModule.swerveIDToName(i) + " Translation Position",
          swerveModules[i]::getWheelPosition,
          null);
      builder.addDoubleProperty(
          SwerveModule.swerveIDToName(i) + " Rotation Angle", swerveModules[i]::getAngle, null);
    }
  }
}
