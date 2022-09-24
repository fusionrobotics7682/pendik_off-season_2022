// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.triggers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class TimerTrigger extends Trigger {

  Timer timer = new Timer();

  // Start timer when the reference type is supported
  public boolean getTimerTrigger(double currentValue, double setpointValue) {
    if(currentValue <= setpointValue+3 && currentValue >= setpointValue-3){
      timer.start();
      if(timer.get() >= 3){
        timer.stop();
        return true;
      }
      return false;
    }
    timer.reset();
    return false;
  }
}
