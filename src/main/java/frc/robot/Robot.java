// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auton.Autons;
import frc.robot.auton.SwervePositionController;
import frc.robot.auton.Trajectories;
import frc.robot.commands.drivetrain.AlignPitch;
import frc.robot.commands.drivetrain.SwerveManual;
import frc.robot.commands.elevator.ElevatorManual;
import frc.robot.subsystems.AngledElevator;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.CameraPoseEstimation;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private SendableChooser<String> autonChooser;

  @Override
  public void robotInit() {
    LiveWindow.setEnabled(true);
    LiveWindow.enableAllTelemetry();
    SmartDashboard.putData(RobotMap.Field.FIELD);
    SmartDashboard.putBoolean("red", Trajectories.isFlipped());
    CommandScheduler.getInstance().setDefaultCommand(Drivetrain.getInstance(), new SwerveManual());
    CommandScheduler.getInstance()
        .setDefaultCommand(AngledElevator.getInstance(), new ElevatorManual());
    autonChooser = new SendableChooser<String>();
    autonChooser.setDefaultOption("Middle And Cross Path", "Middle And Cross Path");
    autonChooser.addOption("Middle Path", "Middle Path");
    autonChooser.addOption("Bottom Path", "Bottom Path");
    autonChooser.addOption("Top Path", "Top Path");
    autonChooser.addOption("No auton", "No auton");
    SmartDashboard.putData("Auton Chooser", autonChooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    RobotMap.Field.FIELD.setRobotPose(Drivetrain.getInstance().getPoseEstimatorPose2d());
    SmartDashboard.putString("Auton running:", autonChooser.getSelected());
    SmartDashboard.putData(Drivetrain.getInstance());
    SmartDashboard.putData(Claw.getInstance());
    SmartDashboard.putData(AngledElevator.getInstance());
    NetworkTableInstance.getDefault().flushLocal();
    NetworkTableInstance.getDefault().flush();
  }

  @Override
  public void autonomousInit() {
    SmartDashboard.putNumber("X kP", SwervePositionController.X_kP);
    SmartDashboard.putNumber("Y kP", SwervePositionController.Y_kP);
    SmartDashboard.putNumber("Theta kP", SwervePositionController.THETA_kP);
    switch (autonChooser.getSelected()) {
      case "Top Path":
        Drivetrain.getInstance()
            .setPose(Trajectories.apply(new Pose2d(1.91, 4.44, Rotation2d.fromDegrees(180))));
        Autons.topPath.schedule();
        break;
      case "Bottom Path":
        Drivetrain.getInstance()
            .setPose(Trajectories.apply(new Pose2d(1.91, 1.09, Rotation2d.fromDegrees(180))));
        Autons.bottomPath.schedule();
        break;
        // case "Top Path And Push":
        //   Drivetrain.getInstance()
        //       .setPose(Trajectories.apply(new Pose2d(1.91, 4.44, Rotation2d.fromDegrees(180))));
        //   Autons.topPathAndPush.schedule();
        //   break;
        // case "Bottom Path And Push":
        //   Drivetrain.getInstance()
        //       .setPose(Trajectories.apply(new Pose2d(1.91, 1.09, Rotation2d.fromDegrees(180))));
        //   Autons.bottomPathAndPush.schedule();
        //    break;
      case "Middle Path":
        Drivetrain.getInstance()
            .setPose(Trajectories.apply(new Pose2d(1.91, 2.75, Rotation2d.fromDegrees(180))));
        Autons.middlePath.schedule();
        break;
      case "No auton":
         Drivetrain.getInstance().setPose(Trajectories.apply(new Pose2d(1.91, 2.75, Rotation2d.fromDegrees(180))));
         Autons.noAuton.schedule();
         break;
      default:
        Drivetrain.getInstance()
            .setPose(Trajectories.apply(new Pose2d(1.91, 2.75, Rotation2d.fromDegrees(180))));
        Autons.middleAndCross.schedule();
        break;
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    Autons.bottomPath.cancel();
    Autons.topPath.cancel();
    Autons.middlePath.cancel();
    Autons.middleAndCross.cancel();
    Autons.noAuton.cancel();
    Drivetrain.getInstance().setYaw(180);
  }

  @Override
  public void teleopPeriodic() {
    // if (Drivetrain.getInstance().getPoseEstimatorPose2d().getX() > RobotMap.Field.fieldLength/2) {
    //   CameraPoseEstimation.getInstance().setCamPipeline(RobotMap.Field.APRILTAG_INDEX); // check which is reflective tape and which is april tags
    // }
    // else {
    // }
  }



  @Override
  public void disabledInit() {
    AngledElevator.getInstance().setDesiredPosition(0);
    AngledElevator.getInstance().moveToPosition(0);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
