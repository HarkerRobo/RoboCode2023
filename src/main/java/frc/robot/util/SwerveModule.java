package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import harkerrobolib.util.Constants;
import harkerrobolib.util.HSFalconBuilder;
import harkerrobolib.wrappers.HSFalcon;

public class SwerveModule {
  private HSFalcon translation;
  private HSFalcon rotation;

  private CANCoder canCoder;

  private int id;

  private MotorVelocitySystem transLoop;

  // PID Constants
  public static double ROTATION_kP = (RobotMap.IS_COMP) ? 0.0 : 0.3; // TODO
  public static double TRANSLATION_kS = (RobotMap.IS_COMP) ? 0.0 : 0; // TODO
  public static double TRANSLATION_kV = (RobotMap.IS_COMP) ? 0.0 : 0.3; // TODO: tune
  public static double TRANSLATION_kA = (RobotMap.IS_COMP) ? 0.0 : 0.05; // TODO

  public static double TRANSLATION_QELMS = 5;

  public SwerveModule(int id) {
    this.id = id;

    translation =
        new HSFalconBuilder()
            .invert(RobotMap.SwerveModule.TRANSLATION_INVERT[id])
            .supplyLimit(
                RobotMap.SwerveModule.TRANS_MOTOR_CURRENT_PEAK,
                RobotMap.SwerveModule.TRANS_MOTOR_CURRENT_CONTINUOUS,
                RobotMap.SwerveModule.TRANS_MOTOR_CURRENT_PEAK_DUR)
            .build(RobotMap.SwerveModule.TRANSLATION_ID[id], RobotMap.CAN_CHAIN);

    transLoop =
        new MotorVelocitySystem(TRANSLATION_kS, TRANSLATION_kV, TRANSLATION_kA, TRANSLATION_QELMS);

    rotation =
        new HSFalconBuilder()
            .invert(RobotMap.SwerveModule.ROTATION_INVERT[id])
            .statorLimit(
                RobotMap.SwerveModule.ROTATION_MOTOR_CURRENT_PEAK,
                RobotMap.SwerveModule.ROTATION_MOTOR_CURRENT_CONTINUOUS,
                RobotMap.SwerveModule.ROTATION_MOTOR_CURRENT_PEAK_DUR)
            .build(RobotMap.SwerveModule.ROTATION_ID[id], RobotMap.CAN_CHAIN);
    canCoder = new CANCoder(RobotMap.SwerveModule.CAN_CODER_ID[id], RobotMap.CAN_CHAIN);

    initModule();
  }

  private void initModule() {
    setkP(ROTATION_kP);
    setAbsolutePosition();
  }

  public void setAngleAndDrive(SwerveModuleState state) {
    state = optimize(state);
    setDrive(state.speedMetersPerSecond);
    setAngle(state.angle.getDegrees());
    
  }

  private void initSmartDashboard() {
    SmartDashboard.putNumber("Translation Speed", getSpeed());
    SmartDashboard.putNumber("Translation Position", getWheelPosition());

    SmartDashboard.putNumber("Translation kS", TRANSLATION_kS);
    TRANSLATION_kS = SmartDashboard.getNumber("Translation kS", TRANSLATION_kS);
    setkS(TRANSLATION_kS);

    SmartDashboard.putNumber("Translation kV", TRANSLATION_kV);
    TRANSLATION_kV = SmartDashboard.getNumber("Translation kV", TRANSLATION_kV);
    setkV(TRANSLATION_kV);

    SmartDashboard.putNumber("Translation Error", TRANSLATION_QELMS);
    TRANSLATION_QELMS = SmartDashboard.getNumber("Translation Qelms", TRANSLATION_QELMS);
    setQelms(TRANSLATION_QELMS);

    SmartDashboard.putNumber("Rotation Angle", getAngle());
    SmartDashboard.putNumber("Rotation Angle", getAngle());

    SmartDashboard.putNumber("Rotation kP", ROTATION_kP);
    ROTATION_kP = SmartDashboard.getNumber("Rotation kP", ROTATION_kP);
    setkP(ROTATION_kP);
  }

  private SwerveModuleState optimize(SwerveModuleState desiredState) {
    var angle = desiredState.angle.getDegrees();
    var speed = desiredState.speedMetersPerSecond;
    // desiredState.angle.rotateBy(Rotation2d.fromDegrees(getAngle()).unaryMinus());
    while (angle - getAngle() > 180) {
      angle -= 360;
      // delta = desiredState.angle.getDegrees() - getAngle();
    }

    while (angle - getAngle() < -180) {
      angle += 360;
      // delta = desiredState.angle.minus(getAngle());
    }

    if (angle - getAngle() > 90) {
      angle -= 180;
      speed *= -1;
    } else if (angle - getAngle() < -90) {
      angle += 180;
      speed *= -1;
    }
    return new SwerveModuleState(speed, Rotation2d.fromDegrees(angle));
  }
  public void setAngle(double angle){
    rotation.set(ControlMode.Position,angle/RobotMap.SwerveModule.ROTATION_CONVERSION);
  }
  public void setDrive(double drive){
    translation.setVoltage(transLoop.getVoltage(drive, getSpeed()));
  }
  public void setkP(double kP) {
    ROTATION_kP = kP;
    rotation.config_kP(Constants.SLOT_INDEX, ROTATION_kP);
  }

  public void setkS(double kS) {
    TRANSLATION_kS = kS;
    transLoop =
        new MotorVelocitySystem(TRANSLATION_kS, TRANSLATION_kV, TRANSLATION_kA, TRANSLATION_QELMS);
  }

  public void setkV(double kV) {
    TRANSLATION_kV = kV;
    transLoop =
        new MotorVelocitySystem(TRANSLATION_kS, TRANSLATION_kV, TRANSLATION_kA, TRANSLATION_QELMS);
  }

  public void setQelms(double error) {
    TRANSLATION_QELMS = error;
    transLoop =
        new MotorVelocitySystem(TRANSLATION_kS, TRANSLATION_kV, TRANSLATION_kA, TRANSLATION_QELMS);
  }

  private void setAbsolutePosition() {
    double pos = canCoder.getAbsolutePosition() - RobotMap.SwerveModule.CAN_CODER_OFFSETS[id];
    rotation.setSelectedSensorPosition(pos / RobotMap.SwerveModule.ROTATION_CONVERSION);
    zeroTranslation();
  }

  public void zeroTranslation() {
    translation.setSelectedSensorPosition(0);
  }

  public double getAngle() {
    return rotation.getSelectedSensorPosition() * RobotMap.SwerveModule.ROTATION_CONVERSION;
  }

  public double getSpeed() {
    return translation.getSelectedSensorVelocity() * RobotMap.SwerveModule.VELOCITY_CONVERSION;
  }

  public double getWheelPosition() {
    return translation.getSelectedSensorPosition() * RobotMap.SwerveModule.POSITION_CONVERSION;
  }

  public static String swerveIDToName(int swerveID) {
    String output = "";
    if (swerveID < 2) output += "Front ";
    else output += "Back ";
    if (swerveID % 2 == 0) output += "Left";
    else output += "Right";
    return output;
  }

  public SwerveModulePosition getSwerveModulePosition() {
    return new SwerveModulePosition(getWheelPosition(), Rotation2d.fromDegrees(getAngle()));
  }

  public SwerveModuleState getSwerveModuleState() {
    return new SwerveModuleState(getSpeed(), Rotation2d.fromDegrees(getAngle()));
  }

}
