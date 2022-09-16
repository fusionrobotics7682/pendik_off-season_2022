// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

  Victor climb1 = new Victor(Constants.ClimbConstants.SHOOTER_1_PIN);

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climb1(){
    climb1.set(1);
  }

  public void release(){
    climb1.set(-1);
  }

  public void stop(){
    climb1.set(0);
  }
}
