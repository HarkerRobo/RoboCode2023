package frc.robot.util;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.CANCoder;

import frc.robot.RobotMap;
import harkerrobolib.wrappers.HSFalcon;

public class SwerveModule {
    private HSFalcon translation;
    private HSFalcon rotation;

    private CANCoder canCoder;

    private int id; // swerveID

    public SwerveModule(int id) {
        this.id = id;

        translation = new HSFalcon(RobotMap.SwerveModule.TRANSLATION_ID[id], RobotMap.CAN_CHAIN);
        rotation = new HSFalcon(RobotMap.SwerveModule.ROTATION_ID[id], RobotMap.CAN_CHAIN);
        canCoder = new CANCoder(RobotMap.SwerveModule.CAN_CODER_ID[id], RobotMap.CAN_CHAIN);

        motorInit();
    }

    public HSFalcon getMotor() {
        return translation;
    }

    public void setAngleAndDrive(double angle, double drive) {
    }

    public void motorInit() {
        // Factory Default
        translation.configFactoryDefault();
        rotation.configFactoryDefault();

        // Inverts
        translation.setInverted(RobotMap.SwerveModule.TRANSLATION_INVERT[id]);
        rotation.setInverted(RobotMap.SwerveModule.ROTATION_INVERT[id]);

        // Feedback Sensor
        translation.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rotation.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        // Current Limits
        translation.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration());
        rotation.configStatorCurrentLimit(new StatorCurrentLimitConfiguration());

        // Voltage Comp
        translation.enableVoltageCompensation(true);
        rotation.enableVoltageCompensation(true);
        translation.configVoltageCompSaturation(RobotMap.MAX_CONTROL_EFFORT);
        rotation.configVoltageCompSaturation(RobotMap.MAX_CONTROL_EFFORT);

        // Velocity Measurement Window | Sensitivity / Rolling Average
        translation.configVelocityMeasurementWindow(RobotMap.SwerveModule.VELOCITY_FILTER);

        // Neutral Mode
        rotation.setNeutralMode(NeutralMode.Brake);

        // PID
        rotation.selectProfileSlot(RobotMap.SwerveModule.SLOT_INDEX, RobotMap.SwerveModule.PID_INDEX);
        rotation.config_kP(RobotMap.SwerveModule.SLOT_INDEX, RobotMap.SwerveModule.kP);
        rotation.config_kI(RobotMap.SwerveModule.SLOT_INDEX, RobotMap.SwerveModule.kP);
        rotation.config_kD(RobotMap.SwerveModule.SLOT_INDEX, RobotMap.SwerveModule.kP);
    }
}