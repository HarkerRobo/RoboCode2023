package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import harkerrobolib.util.HSFalconBuilder;
import harkerrobolib.wrappers.HSFalcon;

public class Arm extends SubsystemBase {
    private static Arm instance;
    
    private HSFalcon extensionMotor;
    private HSFalcon angleMotor;

    public double EXTENSION_KP = 0;
    public double ANGLE_KP = 0;

    private Arm(){
        extensionMotor =
            new HSFalconBuilder()
                .invert(RobotMap.Arm.EXTENSION_INVERTED)
                .supplyLimit(
                    RobotMap.Arm.EXTENSION_CURRENT_PEAK,
                    RobotMap.Arm.EXTENSION_CURRENT_CONTINOUS,
                    RobotMap.Arm.EXTENSION_CURRENT_PEAK_DUR)
                .build(RobotMap.Arm.EXTENSION_ID, RobotMap.CAN_CHAIN);

        angleMotor =
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
        angleMotor.set(ControlMode.Position, angle / RobotMap.Arm.CONVERSION_ANGLE);
    }

    private void initMotors() {
        addChild("Extension Motor", extensionMotor);
        addChild("Angle Motor", angleMotor);
        setkP();
    }

    private void setkP() {
        extensionMotor.config_kP(RobotMap.Arm.SLOT_INDEX, EXTENSION_KP);
        angleMotor.config_kP(RobotMap.Arm.SLOT_INDEX, ANGLE_KP);
    }

    public boolean checkAngle(double desired) {
        if (desired == getExtension()) {
            return true;
        }

        return false;
    }

    public boolean checkExtend(double desired) {
        if (desired == getAngle()) {
            return true;
        }
        
        return false;
    }

    public double getExtension() {
        return extensionMotor.getSelectedSensorPosition() * RobotMap.Arm.CONVERSION_POSITION;
    }

    public double getAngle() {
        return angleMotor.getSelectedSensorPosition() * RobotMap.Arm.CONVERSION_ANGLE;
    }

    public static Arm getInstance() {
        if (instance == null){
            instance = new Arm();
        }
        return instance;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("extension", )
    }

        
}