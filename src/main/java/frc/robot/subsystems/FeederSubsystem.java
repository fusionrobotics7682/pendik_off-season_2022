// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {

  Victor feeder1 = new Victor(Constants.FeederConstants.FEEDER_1_PIN);
  Victor feeder2 = new Victor(Constants.FeederConstants.FEEDER_2_PIN);

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void getIn(){
    feeder1.set(1);
    feeder2.set(1);
  }

  public void getOut(){
    feeder1.set(-1);
    feeder2.set(-1);
  }
  
  public void stop(){
    feeder1.set(0);
    feeder2.set(0);
  }
}
