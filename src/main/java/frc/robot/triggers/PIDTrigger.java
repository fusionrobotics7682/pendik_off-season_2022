package frc.robot.triggers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class PIDTrigger extends Trigger {

  public boolean getMathTrigger(double currentValue, double setpointValue){
    /*collect data -> do statistics(arithmetics average) -> return bool value*/
    return true;
  }
}
