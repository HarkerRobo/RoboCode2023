package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Claw extends SubsystemBase {
    private static Claw instance;

    private DoubleSolenoid claw;

    private Claw() {
        claw = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotMap.Claw.CLAW_FOWARD_ID, RobotMap.Claw.CLAW_REVERSE_ID);
    }

    public void toggleClaw() {
        if (claw.get() == Value.kForward) {
            claw.set(Value.kReverse);
        }
        else {
            claw.set(Value.kForward);
        }
    }

    public static Claw getInstance() {
        if (instance == null) instance = new Claw();
        return instance;
    }
}
