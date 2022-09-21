package frc.robot.frc_auto_core;

import edu.wpi.first.math.controller.PIDController;

public class AutoConfigurer {
    
    PIDController pidController;

    public PIDController getPidController(double kP, double kI, double kD){
        pidController = new PIDController(kP, kI, kD);
        return pidController;
    }
    
}
