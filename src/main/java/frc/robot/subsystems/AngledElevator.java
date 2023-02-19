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

  private static double kP = 0; // TODO
  private static double kG = 0; // kS + kG //TODO

  private static double MAX_ERROR = 0; // TODO

  private DigitalInput limitSwitch;

  private static double CRUISE_VELOCITY = 0; //TODO
  private static double CRUISE_ACCELERATION = 0; //TODO

  private AngledElevator() {
    master =
        new HSFalconBuilder()
            .invert(RobotMap.AngledElevator.MASTER_INVERTED)
            .supplyLimit(
                RobotMap.AngledElevator.MASTER_CURRENT_PEAK,
                RobotMap.AngledElevator.MASTER_CURRENT_CONTINOUS,
                RobotMap.AngledElevator.MASTER_CURRENT_PEAK_DUR)
            .build(RobotMap.AngledElevator.MASTER_ID, RobotMap.CAN_CHAIN);

    follower =
        new HSFalconBuilder()
            .invert(RobotMap.AngledElevator.FOLLOWER_INVERTED)
            .supplyLimit(
                RobotMap.AngledElevator.FOLLOWER_CURRENT_PEAK,
                RobotMap.AngledElevator.FOLLOWER_CURRENT_CONTINOUS,
                RobotMap.AngledElevator.FOLLOWER_CURRENT_PEAK_DUR)
            .build(RobotMap.AngledElevator.FOLLOWER_ID, RobotMap.CAN_CHAIN);
    initMotors();
  }

  public void moveToPosition(double height) {
    double signError = Math.signum(height - getPosition());
    double percentOutput = signError * (kG) / Constants.MAX_VOLTAGE;
    master.set(ControlMode.MotionMagic, height, DemandType.ArbitraryFeedForward, percentOutput);
  }

  private void initMotors() {
    addChild("Master Motor", master);
    addChild("Follower Motor", follower);
    addChild("Limit Switch", limitSwitch);
    // master.configForwardSoftLimitThreshold(MAX_ERROR);
    follower.follow(master);
    setkP(kP);
    master.configMotionCruiseVelocity(CRUISE_VELOCITY);
    master.configMotionAcceleration(CRUISE_ACCELERATION);
  }

  private void setkP(double newkP) {
    kP = newkP;
    master.config_kP(Constants.SLOT_INDEX, kP);
  }

  private void setkG(double newkG) {
    kG = newkG;
  }

  public void addToPositions(double value) {
    RobotMap.AngledElevator.POSITIONS[0] += value;
    RobotMap.AngledElevator.POSITIONS[1] += value;
    RobotMap.AngledElevator.POSITIONS[2] += value;
    RobotMap.AngledElevator.POSITIONS[3] += value;
  }

  public boolean checkExtend(double desired) {
    return Math.abs(desired - getPosition()) < MAX_ERROR;
  }

  public double getPosition() {
    return master.getSelectedSensorPosition();
  }

  public double getkP() {
    return kP;
  }

  public double getkG() {
    return kG;
  }

  public void setExtensionPower(double power) {
    master.set(ControlMode.PercentOutput, power);
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
    builder.addDoubleProperty("Position", this::getPosition, this::moveToPosition);
    builder.addDoubleProperty("Elevator kP", this::getkP, this::setkP);
    builder.addDoubleProperty("Elevator kG", this::getkG, this::setkG);
  }
}
