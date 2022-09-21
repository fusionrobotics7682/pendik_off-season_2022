package frc.robot.frc_auto_core;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;

public class VelocityController {
    
    AutoConfigurer autoConfigurer;
    Encoder encoder;
    Timer timer = new Timer();
    double output = 0;

    public VelocityController(AutoConfigurer autoConfigurer, Encoder encoder) {
        this.autoConfigurer = autoConfigurer;
        this.encoder = encoder;
        timer.reset();
        timer.start();
    }

    public double goXVelocity(double setpointVelocity){
        double currentPosition = Units.feetToMeters(encoder.get()*AutoConstants.K_DRIVE_TICK_2_FEET);
        double currentVelocity = currentPosition/timer.get();

        output += autoConfigurer.getPidController(Constants.DriveConstants.ENCODER_KP, Constants.DriveConstants.ENCODER_KI, Constants.DriveConstants.ENCODER_KD).calculate(currentVelocity, setpointVelocity);

        return output;
    }

}