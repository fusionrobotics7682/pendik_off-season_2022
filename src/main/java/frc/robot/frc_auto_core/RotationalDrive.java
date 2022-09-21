package frc.robot.frc_auto_core;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.constants.Constants;

public class RotationalDrive {
    
    private AutoConfigurer autoConfigurer;
    private Gyro gyro;
    double temp;

    public RotationalDrive(AutoConfigurer autoConfigurer, Gyro gyro) {
        this.autoConfigurer = autoConfigurer;
        this.gyro = gyro;
        temp = gyro.getAngle();
    }

    public double turnXDegrees(double degreesSetpoint){
        double output = autoConfigurer.getPidController(Constants.DriveConstants.GYRO_KP, Constants.DriveConstants.GYRO_KI, Constants.DriveConstants.GYRO_KD).calculate(gyro.getAngle(), degreesSetpoint);
        return output;
    }

}
