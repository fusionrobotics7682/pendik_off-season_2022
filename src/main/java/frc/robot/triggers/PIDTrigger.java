package frc.robot.triggers;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class PIDTrigger extends Trigger {
  
  LinearFilter filter = LinearFilter.movingAverage(50);

  public double checkPidTrigger(double currentValue){
    return filter.calculate(currentValue);
  }
}
