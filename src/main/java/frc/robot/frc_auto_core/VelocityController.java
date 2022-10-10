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

    public double runShooterXVelocity(double setpointVelocity){
        double rpm = relativeEncoder.getVelocity();
        double velocity = 2* Math.PI * rpm / 60 * 0.22965879265;     
        output += autoConfigurer.getPidController(
            Constants.DriveConstants.ENCODER_KP,
            Constants.DriveConstants.ENCODER_KI,
            Constants.DriveConstants.ENCODER_KD)
        .calculate(velocity, setpointVelocity);
        return output;
    }

}