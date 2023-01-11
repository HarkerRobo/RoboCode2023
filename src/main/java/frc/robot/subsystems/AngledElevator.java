package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import harkerrobolib.util.HSFalconBuilder;
import harkerrobolib.wrappers.HSFalcon;

public class AngledElevator extends SubsystemBase
{
    private static AngledElevator instance;

    private HSFalcon master;
    private HSFalcon follower;

    private AngledElevator()
    {
        master = new HSFalconBuilder()
                .invert(RobotMap.AngledElevator.MASTER_INVERTED)
                .supplyLimit(
                    RobotMap.AngledElevator.MASTER_CURRENT_PEAK,
                    RobotMap.AngledElevator.MASTER_CURRENT_CONTINOUS,
                    RobotMap.AngledElevator.MASTER_CURRENT_PEAK_DUR
                )
                .build(RobotMap.AngledElevator.MASTER_ID, RobotMap.CAN_CHAIN);
        
        follower = new HSFalconBuilder()
                .invert(RobotMap.AngledElevator.FOLLOWER_INVERTED)
                .supplyLimit(
                    RobotMap.AngledElevator.FOLLOWER_CURRENT_PEAK,
                    RobotMap.AngledElevator.FOLLOWER_CURRENT_CONTINOUS,
                    RobotMap.AngledElevator.FOLLOWER_CURRENT_PEAK_DUR
                )
                .build(RobotMap.AngledElevator.FOLLOWER_ID, RobotMap.CAN_CHAIN);
        initMotors();
    }

    private void initMotors()
    {
        follower.follow(master);
    }
    public double getDistance(double encoderMeasurement)
    {
        return encoderMeasurement;
    }

    public HSFalcon getMaster()
    {
        return master;
    }

    public HSFalcon getFollower()
    {
        return follower;
    }

    public static AngledElevator getInstance()
    {
        if (instance == null)
        {
            return new AngledElevator();
        }
        return instance;
    }
}
