package frc.robot.auton;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import java.util.function.Supplier;

public class SwervePositionController extends CommandBase {
  // TODO
  public static double X_kP = (RobotMap.IS_COMP) ? 0.0 : 2.7;
  public static double X_kI = (RobotMap.IS_COMP) ? 0.0 : 0.0;
  public static double X_kD = (RobotMap.IS_COMP) ? 0.0 : 0.0;

  public static double Y_kP = (RobotMap.IS_COMP) ? 0.0 : 4.5;
  public static double Y_kI = (RobotMap.IS_COMP) ? 0.0 : 0.0;
  public static double Y_kD = (RobotMap.IS_COMP) ? 0.0 : 0.0;

  public static double THETA_kP = (RobotMap.IS_COMP) ? 0.0 : 3.5;
  public static double THETA_kI = (RobotMap.IS_COMP) ? 0.0 : 0.0;
  public static double THETA_kD = (RobotMap.IS_COMP) ? 0.0 : 0.0;

  private static PIDController xController = new PIDController(X_kP, X_kI, X_kD);
  private static PIDController yController = new PIDController(Y_kP, Y_kI, Y_kD);
  private static ProfiledPIDController thetaController =
      new ProfiledPIDController(
          THETA_kP,
          THETA_kI,
          THETA_kD,
          new Constraints(RobotMap.MAX_ANGLE_VELOCITY, RobotMap.MAX_ANGLE_ACCELERATION));

  private final Trajectory trajectory;
  private final Supplier<Rotation2d> refHeading;
  private final Supplier<Rotation2d> startHeading;
  private final Timer timer = new Timer();

  public SwervePositionController(
      Trajectory trajectory, Supplier<Rotation2d> refHeading, Supplier<Rotation2d> startHeading) {
    this.trajectory = trajectory;
    this.refHeading = refHeading;
    this.startHeading = startHeading;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(Drivetrain.getInstance());
  }

  public SwervePositionController(Trajectory trajectory, Supplier<Rotation2d> refHeading) {
    this(trajectory, refHeading, null);
  }

  @Override
  public void initialize() {
    Drivetrain.getInstance().setPose(trajectory.getInitialPose());
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    X_kP = SmartDashboard.getNumber("X kP", X_kP);
    Y_kP = SmartDashboard.getNumber("Y kP", Y_kP);
    THETA_kP = SmartDashboard.getNumber("Theta kP", THETA_kP);
    SmartDashboard.putNumber("X kP", X_kP);
    SmartDashboard.putNumber("Y kP", Y_kP);
    SmartDashboard.putNumber("Theta kP", THETA_kP);
    xController.setP(X_kP);
    yController.setP(Y_kP);
    thetaController.setP(THETA_kP);

    Trajectory.State goal = Trajectories.apply(trajectory.sample(timer.get()));
    double xFF = goal.velocityMetersPerSecond * goal.poseMeters.getRotation().getCos();
    double yFF = goal.velocityMetersPerSecond * goal.poseMeters.getRotation().getSin();
    Rotation2d angleRef = Trajectories.apply(refHeading.get());

    Pose2d currentPose = Trajectories.apply(Drivetrain.getInstance().getPoseEstimatorPose2d());
    double clampAdd =
        1
            + Math.abs(angleRef.getRadians() - currentPose.getRotation().getRadians())
                * (2 / Math.PI);
    double thetaFF =
        MathUtil.clamp(
            thetaController.calculate(
                currentPose.getRotation().getRadians(), angleRef.getRadians()),
            -clampAdd,
            clampAdd);
    // poseError = poseRef.relativeTo(currentPose);
    // Rotation2d rotationError = angleRef.minus(currentPose.getRotation());

    // Calculate feedback velocities (based on position error).
    double xFeedback = xController.calculate(currentPose.getX(), goal.poseMeters.getX());
    double yFeedback = yController.calculate(currentPose.getY(), goal.poseMeters.getY());

    // Return next output.
    ChassisSpeeds adjustedSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback, yFF + yFeedback, thetaFF, currentPose.getRotation());
    Drivetrain.getInstance().setAngleAndDrive(adjustedSpeeds);
    // Pose2d poseError = autonomusController.m_poseError;
    // Rotation2d rotError = autonomusController.m_rotationError;
    SmartDashboard.putNumber("goal X", goal.poseMeters.getX());
    SmartDashboard.putNumber("goal Y", goal.poseMeters.getY());
    SmartDashboard.putNumber("goal theta", angleRef.getDegrees());
    // SmartDashboard.putNumber("Traj-X-Error", Units.metersToInches(poseError.getX()));
    // SmartDashboard.putNumber("Traj-Y-Error", Units.metersToInches(poseError.getY()));
    // SmartDashboard.putNumber("Traj-Theta-Error", rotationError.getDegrees());
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= trajectory.getTotalTimeSeconds();
  }

  @Override
  public void end(boolean interrupted) {
    Drivetrain.getInstance().setAngleAndDrive(new ChassisSpeeds());
  }
}
