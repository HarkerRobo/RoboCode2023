package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import harkerrobolib.util.HSFalconBuilder;
import harkerrobolib.wrappers.HSFalcon;

public class Arm extends SubsystemBase {
    private static Arm instance;
    
    private HSFalcon extensionMotor;
    private HSFalcon rotationMotor;

    private static double EXTENSION_kP = 0;
    private static double ROTATION_kP = 0;

    private static double MAX_EXTENSION_ERROR = 0;
    private static double MAX_ROTATION_ERROR = 0;

    private DigitalInput extensionLimitSwitch;
    private DigitalInput rotationLimitSwitch;

    private Arm(){
        extensionMotor =
            new HSFalconBuilder()
                .invert(RobotMap.Arm.EXTENSION_INVERTED)
                .supplyLimit(
                    RobotMap.Arm.EXTENSION_CURRENT_PEAK,
                    RobotMap.Arm.EXTENSION_CURRENT_CONTINOUS,
                    RobotMap.Arm.EXTENSION_CURRENT_PEAK_DUR)
                .build(RobotMap.Arm.EXTENSION_ID, RobotMap.CAN_CHAIN);

        rotationMotor =
            new HSFalconBuilder()
                .invert(RobotMap.Arm.ANGLE_INVERTED)
                .supplyLimit(
                    RobotMap.Arm.ANGLE_CURRENT_PEAK,
                    RobotMap.Arm.ANGLE_CURRENT_CONTINOUS,
                    RobotMap.Arm.ANGLE_CURRENT_PEAK_DUR)
                .build(RobotMap.Arm.ANGLE_ID, RobotMap.CAN_CHAIN);
        initMotors();
    }

    public void extendToPosition(double height) {
        extensionMotor.set(ControlMode.Position, height / RobotMap.Arm.CONVERSION_POSITION);
    }

    public void rotateToAngle(double angle)
    {
        rotationMotor.set(ControlMode.Position, angle / RobotMap.Arm.CONVERSION_ANGLE);
    }

    private void initMotors() {
        addChild("Extension Motor", extensionMotor);
        addChild("Angle Motor", rotationMotor);
        addChild("Extension Limit Switch",extensionLimitSwitch);
        addChild("Rotation Limit Switch", rotationLimitSwitch);

        setExtensionkP(EXTENSION_kP);
        setRotationkP(ROTATION_kP);
    }

    private void setExtensionkP(double kP) {
        EXTENSION_kP = kP;
        extensionMotor.config_kP(RobotMap.Arm.SLOT_INDEX, EXTENSION_kP);
    }

    private void setRotationkP(double kP) {
        ROTATION_kP = kP;
        rotationMotor.config_kP(RobotMap.Arm.SLOT_INDEX, ROTATION_kP);
    }

    public boolean checkAngle(double desired) {
        return Math.abs(getRotationError(desired)) < MAX_ROTATION_ERROR;

    }

    public boolean checkExtend(double desired) {
        return Math.abs(getExtensionError(desired)) < MAX_EXTENSION_ERROR;
    }

    private double getExtensionError(double desired) {
        return desired - getExtension();
    }

    private double getRotationError(double desired) {
        return desired - getAngle();
    }

    public double getExtension() {
        return extensionMotor.getSelectedSensorPosition() * RobotMap.Arm.CONVERSION_POSITION;
    }

    public double getAngle() {
        return rotationMotor.getSelectedSensorPosition() * RobotMap.Arm.CONVERSION_ANGLE;
    }

    public double getRotationkP() {
        return ROTATION_kP;
    }

    public double getExtensionkP() {
        return EXTENSION_kP;
    }
    public void setExtensionPower(double power){
        extensionMotor.set(ControlMode.PercentOutput, power);
      }
    public void setRotationPower(double power){
        rotationMotor.set(ControlMode.PercentOutput, power);
    }
    public boolean extensionStop(){
        return !extensionLimitSwitch.get();
    }
    public boolean rotationStop(){
        return !rotationLimitSwitch.get();
    }

    public static Arm getInstance() {
        if (instance == null){
            instance = new Arm();
        }
        return instance;
    }
    

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Arm");
        builder.addDoubleProperty("Extension Position", this::getExtension, this::extendToPosition);
        builder.addDoubleProperty("Extension kP", this::getExtensionkP, this::setExtensionkP);

        builder.addDoubleProperty("Rotation Angle", this::getAngle, this::rotateToAngle);
        builder.addDoubleProperty("Rotation kP", this::getRotationkP, this::setRotationkP);

    }

        
}