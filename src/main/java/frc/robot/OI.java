package frc.robot;

import harkerrobolib.joysticks.XboxGamepad;

public class OI {
    private static OI instance;
    
    private XboxGamepad driver;
    private XboxGamepad operator;
    private OI() {
        driver = new XboxGamepad(RobotMap.OI.DRIVER_ID);
        operator = new XboxGamepad(RobotMap.OI.OPERATOR_ID);
        initBindings();
    }

    private void initBindings() {
    
    }

    private static OI getInstance() {
        if (instance == null) instance = new OI();
        return instance;
    }
    
}
