package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

  private static double PIGEON_kP = 0.007;

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
    
    Pose2d initalPoseMeters = new Pose2d();
    
    poseEstimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.fromDegrees(getHeading()), getModulePositions(), initalPoseMeters);

    }

  public double adjustPigeon(double omega) {
    if (Math.abs(omega) <= RobotMap.Drivetrain.MIN_OUTPUT)
        omega = -PIGEON_kP * (getHeading() - prevHeading);
    else prevHeading = getHeading();

    return omega;
  }
    
  public double getHeading() {
    return (RobotMap.IS_PIGEON_UP) ? -pigeon.getYaw() : pigeon.getYaw();
  }

  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(getHeading());
  }

  private SwerveModulePosition[] getModulePositions()
  {
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
  }
  public static Drivetrain getInstance() {
    if (instance == null) instance = new Drivetrain();
    return instance;
  }
  
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }


}
