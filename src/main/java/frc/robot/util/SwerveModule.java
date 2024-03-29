package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
  public static double ROTATION_kP = 0.25;
  public static double TRANSLATION_kS = 0.00; // 0.02569;
  public static double TRANSLATION_kV = 1.954584;
  public static double TRANSLATION_kA = 0.21522;

  public static final double TRANSLATION_QELMS = 15;

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
    canCoder = new CANCoder(RobotMap.SwerveModule.CAN_CODER_ID[id]);

    initModule();
  }


  private void initModule() {
    rotation.config_kP(Constants.SLOT_INDEX, ROTATION_kP);
    translation.enableVoltageCompensation(false);
    translation.configVelocityMeasurementWindow(32);
    canCoder.configFactoryDefault();
    canCoder.clearStickyFaults();
    canCoder.configSensorInitializationStrategy(
        SensorInitializationStrategy.BootToAbsolutePosition);
    canCoder.setPositionToAbsolute();
    canCoder.configSensorDirection(false);
    setAbsolutePosition();
  }

  public void setAngleAndDrive(SwerveModuleState state) {
    state = optimize(state);
    translation.setVoltage(transLoop.getVoltage(state.speedMetersPerSecond, getSpeed()));
    rotation.set(
        ControlMode.Position, state.angle.getDegrees() / RobotMap.SwerveModule.ROTATION_CONVERSION);
  }

  private SwerveModuleState optimize(SwerveModuleState desiredState) {
    double currentAngle = Math.toRadians(getAngle());
    double targetAngleSetpoint = Math.IEEEremainder(desiredState.angle.getRadians(), Math.PI * 2);
    double remainder =  currentAngle % (Math.PI * 2);
    var adjusted = targetAngleSetpoint + (currentAngle - remainder);

    var speed = desiredState.speedMetersPerSecond;
    SmartDashboard.putNumber(swerveIDToName(id) + " Desired Translation Speed", speed);
    // desiredState.angle.rotateBy(Rotation2d.fromDegrees(getAngle()).unaryMinus());

    
    if (adjusted - currentAngle > Math.PI) {
      adjusted -= Math.PI * 2;
      // delta = desiredState.angle.getDegrees() - getAngle();
    }

    if (adjusted - currentAngle < -Math.PI) {
      adjusted += Math.PI * 2;
      // delta = desiredState.angle.minus(getAngle());
    }

    if (adjusted - currentAngle > Math.PI / 2) {
      adjusted -= Math.PI;
      speed *= -1;
    } else if (adjusted - currentAngle < -Math.PI / 2) {
      adjusted += Math.PI;
      speed *= -1;
    }
    return new SwerveModuleState(speed, Rotation2d.fromRadians(adjusted));
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
