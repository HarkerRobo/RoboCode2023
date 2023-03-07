package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import harkerrobolib.util.Constants;
import harkerrobolib.util.HSFalconBuilder;
import harkerrobolib.wrappers.HSFalcon;

public class AngledElevator extends SubsystemBase {
  private static AngledElevator instance;

  private HSFalcon master;
  private HSFalcon follower;

  private static final double kP = 0.15;
  private static final double kG = 0.085;

  private static final double MAX_ERROR = 550;

  private double prevPosition;

  private DigitalInput limitSwitch;

  private static final double CRUISE_VELOCITY = 7447;
  private static final double CRUISE_ACCELERATION = 4447;

  private AngledElevator() {
    master =
        new HSFalconBuilder()
            .invert(RobotMap.AngledElevator.MASTER_INVERTED)
            // .statorLimit(
            //     RobotMap.AngledElevator.MASTER_CURRENT_PEAK,
            //     RobotMap.AngledElevator.MASTER_CURRENT_CONTINOUS,
            //     RobotMap.AngledElevator.MASTER_CURRENT_PEAK_DUR)
            .build(RobotMap.AngledElevator.MASTER_ID, RobotMap.CAN_CHAIN);

    follower =
        new HSFalconBuilder()
            .invert(RobotMap.AngledElevator.FOLLOWER_INVERTED)
            // .statorLimit(
            //     RobotMap.AngledElevator.FOLLOWER_CURRENT_PEAK,
            //     RobotMap.AngledElevator.FOLLOWER_CURRENT_CONTINOUS,
            //     RobotMap.AngledElevator.FOLLOWER_CURRENT_PEAK_DUR)
            .build(RobotMap.AngledElevator.FOLLOWER_ID, RobotMap.CAN_CHAIN);
    limitSwitch = new DigitalInput(RobotMap.AngledElevator.LIMIT_SWTICH_ID);
    initElevator();
  }

  public void moveToPosition(double height) {
    moveToPosition(height, false);
  }

  public void moveToPosition(double desired, boolean set) {
    if (set) prevPosition = desired;
    master.set(ControlMode.MotionMagic, prevPosition);
  }

  private void initElevator() {
    addChild("Master Motor", master);
    addChild("Follower Motor", follower);
    addChild("Limit Switch", limitSwitch);
    follower.follow(master);
    master.config_kP(Constants.SLOT_INDEX, kP);
    master.configForwardSoftLimitThreshold(RobotMap.AngledElevator.FORWARD_LIMIT);
    master.configReverseSoftLimitThreshold(RobotMap.AngledElevator.REVERSE_LIMIT);
    master.configReverseSoftLimitEnable(true);
    master.configForwardSoftLimitEnable(true);
    master.overrideSoftLimitsEnable(true);
    master.configMotionCruiseVelocity(CRUISE_VELOCITY);
    master.configMotionAcceleration(CRUISE_ACCELERATION);
    master.configClosedloopRamp(0.01);
    prevPosition = 0;
  }

  public void addToPositions(double value) {
    RobotMap.AngledElevator.POSITIONS[0] += value;
    RobotMap.AngledElevator.POSITIONS[1] += value;
    RobotMap.AngledElevator.POSITIONS[2] += value;
    RobotMap.AngledElevator.POSITIONS[3] += value;
  }

  public boolean checkExtend(double desired) {
    return Math.abs(desired - master.getSelectedSensorPosition()) < MAX_ERROR;
  }

  public double getPosition() {
    return master.getSelectedSensorPosition();
  }

  public double getkP() {
    return kP;
  }

  public void setExtensionPower(double power) {
    if (power == 0) master.neutralOutput();
    else master.set(ControlMode.PercentOutput, power, DemandType.ArbitraryFeedForward, kG);
  }

  public void resetEncoders() {
    master.setSelectedSensorPosition(0);
    follower.setSelectedSensorPosition(0);
  }

  public boolean extensionStop() {
    return !limitSwitch.get();
  }

  public static AngledElevator getInstance() {
    if (instance == null) {
      instance = new AngledElevator();
    }
    return instance;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Elevator");
    builder.setActuator(true);
    builder.setSafeState(() -> setExtensionPower(0));
    builder.addDoubleProperty("Position", this::getPosition, this::moveToPosition);
  }
}
