// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auton.Autons;
import frc.robot.auton.SwervePositionController;
import frc.robot.commands.drivetrain.AlignPitch;
import frc.robot.commands.drivetrain.AlignYaw;
import frc.robot.commands.drivetrain.SwerveManual;
import frc.robot.commands.elevator.ElevatorManual;
import frc.robot.subsystems.AngledElevator;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;

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
  private SendableChooser<CommandBase> autonChooser;

  @Override
  public void robotInit() {
    LiveWindow.setEnabled(true);
    SmartDashboard.putData(RobotMap.Field.FIELD);
    SmartDashboard.putNumber("Pitch kP", AlignPitch.kP);
    SmartDashboard.putNumber("Pigeon kP", Drivetrain.PIGEON_kP);
    SmartDashboard.putNumber("Yaw kP", AlignYaw.kP);
    CommandScheduler.getInstance().setDefaultCommand(Drivetrain.getInstance(), new SwerveManual());
    // CommandScheduler.getInstance()
    //     .setDefaultCommand(AngledElevator.getInstance(), new ElevatorManual());
    autonChooser = new SendableChooser<>();
    autonChooser.setDefaultOption("Middle Path", Autons.middlePath);
    autonChooser.addOption("Bottom Path", Autons.bottomPath);
    autonChooser.addOption("Top Path", Autons.topPath);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putData(Drivetrain.getInstance());
    // SmartDashboard.putData(Claw.getInstance());
    // SmartDashboard.putData(AngledElevator.getInstance());
  }

  @Override
  public void autonomousInit() {
    SmartDashboard.putNumber("X kP", SwervePositionController.X_kP);
    SmartDashboard.putNumber("Y kP", SwervePositionController.Y_kP);
    SmartDashboard.putNumber("Theta kP", SwervePositionController.THETA_kP);
    autonChooser.getSelected().schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

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
