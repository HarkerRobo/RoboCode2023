package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import harkerrobolib.util.HSFalconBuilder;
import harkerrobolib.wrappers.HSFalcon;

public class Claw extends SubsystemBase {
  private static Claw instance;

  private HSFalcon master;

  private Claw() {
    master =
        new HSFalconBuilder()
            .invert(RobotMap.Claw.MASTER_INVERTED)
            .supplyLimit(
                RobotMap.Claw.MASTER_CURRENT_PEAK,
                RobotMap.Claw.MASTER_CURRENT_CONTINOUS,
                RobotMap.Claw.MASTER_CURRENT_PEAK_DUR)
            .build(RobotMap.Claw.MASTER_ID, RobotMap.CAN_CHAIN);
  }

  public void setPercentOutput(double output) {
    master.set(ControlMode.PercentOutput, output);
  }


  public static Claw getInstance() {
    if (instance == null) instance = new Claw();
    return instance;
  }
}
