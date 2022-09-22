package frc.robot.frc_auto_core;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.constants.Constants;

public class VelocityController {
    
    AutoConfigurer autoConfigurer;
    Encoder encoder;
    RelativeEncoder relativeEncoder;
    double output = 0;

    public VelocityController(AutoConfigurer autoConfigurer, Encoder encoder) {
        this.autoConfigurer = autoConfigurer;
        this.encoder = encoder;
    }

    public VelocityController(AutoConfigurer autoConfigurer, RelativeEncoder relativeEncoder) {
        this.autoConfigurer = autoConfigurer;
        this.relativeEncoder = relativeEncoder;
    }

    public double runXVelocity(double setpointVelocity, double currentTime){
        double currentPosition = relativeEncoder.getCountsPerRevolution()/4*Constants.ShooterConstants.K_DRIVE_TICK_2_FEET;
        double currentVelocity = currentPosition/currentTime;

        output += autoConfigurer.getPidController(Constants.DriveConstants.ENCODER_KP, Constants.DriveConstants.ENCODER_KI, Constants.DriveConstants.ENCODER_KD).calculate(currentVelocity, setpointVelocity);
        return output;
    }

}