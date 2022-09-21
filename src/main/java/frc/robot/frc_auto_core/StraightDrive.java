package frc.robot.frc_auto_core;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.constants.Constants;

public class StraightDrive{

    private AutoConfigurer autoConfigurer;

    private Encoder leftEncoder;
    private Encoder rightEncoder;

    public static final double K_DRIVE_TICK_2_FEET = 1.0 / 128 * 6 * Math.PI / 12;

    public StraightDrive(AutoConfigurer autoConfigurer, Encoder leftEncoder, Encoder rightEncoder){
        this.autoConfigurer = autoConfigurer;
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
    }

    public double goXmeter(double meterSetpoint){       
        return calculate(Units.feetToMeters(getEncodersFeetAverage()), meterSetpoint);
    }

    public double goXfeet(double feetSetpoint){
       return calculate(getEncodersFeetAverage(), feetSetpoint);
    }

    public double calculate(double currentDistance, double setpoint){
        return autoConfigurer.getPidController(Constants.DriveConstants.ENCODER_KP, Constants.DriveConstants.ENCODER_KI, Constants.DriveConstants.ENCODER_KD).calculate(currentDistance, setpoint);
    }

    public double getEncodersFeetAverage(){
        return (leftEncoder.get() + rightEncoder.get() / 2) * AutoConstants.K_DRIVE_TICK_2_FEET;
    }
}