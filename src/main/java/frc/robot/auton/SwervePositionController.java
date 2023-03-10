package frc.robot.auton;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import java.util.function.Supplier;

public class SwervePositionController extends CommandBase {
  // TODO
  public static double X_kP = 3.0;
  public static double X_kI = 0.0;
  public static double X_kD = 0.0;

  public static double Y_kP = 2.5;
  public static double Y_kI = 0.0;
  public static double Y_kD = 0.0;

  public static double THETA_kP = 1.0;
  public static double THETA_kI = 0.0;
  public static double THETA_kD = 0.0;

  private static PIDController xController = new PIDController(X_kP, X_kI, X_kD);
  private static PIDController yController = new PIDController(Y_kP, Y_kI, Y_kD);
  private static PIDController thetaController = new PIDController(THETA_kP, THETA_kI, THETA_kD);

  private Trajectory trajectory;
  private Supplier<Rotation2d> refHeading;
  private Supplier<Rotation2d> startHeading;
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
    timer.reset();
    timer.start();
    thetaController.reset();
    xController.reset();
    yController.reset();
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

    Pose2d currentPose = Drivetrain.getInstance().getPoseEstimatorPose2d();
    Rotation2d currentRotation = Drivetrain.getInstance().getRotation();
    double clampAdd =
        1 + Math.abs(angleRef.getRadians() - currentRotation.getRadians()) * (2 / Math.PI);
    double thetaFF =
        MathUtil.clamp(
            thetaController.calculate(currentRotation.getRadians(), angleRef.getRadians()),
            -clampAdd,
            clampAdd);

    SmartDashboard.putNumber("x", currentPose.getX());
    SmartDashboard.putNumber("y", currentPose.getY());
    SmartDashboard.putNumber("rotation pose", currentRotation.getRadians());
    // Calculate feedback velocities (based on position error).
    double xFeedback = xController.calculate(currentPose.getX(), goal.poseMeters.getX());
    double yFeedback = yController.calculate(currentPose.getY(), goal.poseMeters.getY());

    // Return next output.
    ChassisSpeeds adjustedSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback, yFF + yFeedback, thetaFF, currentPose.getRotation());
    Drivetrain.getInstance().setAngleAndDrive(adjustedSpeeds);

    SmartDashboard.putNumber("goal X", goal.poseMeters.getX());
    SmartDashboard.putNumber("goal Y", goal.poseMeters.getY());
    SmartDashboard.putNumber("goal theta", angleRef.getDegrees());
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
