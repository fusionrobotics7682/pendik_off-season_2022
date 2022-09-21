package frc.robot.frc_auto_core;

import java.util.List;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.frc_auto_core.AutoConstants;

public class CircularDrive{

    AutoConfigurer autoConfigurer;

    double leftArcSetpoint;
    double rightArcSetpoint;

    double leftOutput;
    double rightOutput;

    Encoder leftEncoder;
    Encoder rightEncoder;

    double [] arcOutputs = new double [] {0, 0};

    private final double K_DRIVE_TICK_2_FEET = (10.71 / 80 * 6.0 * Math.PI / 12.0)/100;

    public CircularDrive(AutoConfigurer autoConfigurer, Encoder leftEncoder, Encoder rightEncoder){
        this.autoConfigurer = autoConfigurer;
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
    }

    public double [] calculateArcSetpoints(double degrees, double meterSetpoint){
        double temp = 2 * Math.PI * (degrees/360);

        double radius = meterSetpoint / temp;

        if(degrees < 0){
        leftArcSetpoint = 2 * Math.PI * (radius-AutoConstants.DISTANCE_BETWEEN_WHEELS_CM) * (degrees/360);
        rightArcSetpoint = 2 * Math.PI * (radius+AutoConstants.DISTANCE_BETWEEN_WHEELS_CM) * (degrees/360);
        } else{
        leftArcSetpoint = 2 * Math.PI * (radius-AutoConstants.DISTANCE_BETWEEN_WHEELS_CM) * (degrees/360);
        rightArcSetpoint = 2 * Math.PI * (radius+AutoConstants.DISTANCE_BETWEEN_WHEELS_CM) * (degrees/360);    
        }

        rightOutput = autoConfigurer.getPidController(0.5, 0, 0).calculate(Units.feetToMeters(rightEncoder.getDistance()*K_DRIVE_TICK_2_FEET), rightArcSetpoint);
        leftOutput = autoConfigurer.getPidController(0.5, 0, 0).calculate(Units.feetToMeters(leftEncoder.getDistance()*K_DRIVE_TICK_2_FEET), leftArcSetpoint);

        arcOutputs[0] = rightOutput;
        arcOutputs[1] = leftOutput;

        return arcOutputs;
    }

}