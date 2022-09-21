// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class ClimbSubsystem extends SubsystemBase {

  Victor climb1 = new Victor(Constants.ClimbConstants.CLIMB_PIN_1);

  MotorControllerGroup climbControllerGroup = new MotorControllerGroup(climb1);

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climb1(){
    climbControllerGroup.set(1);
  }

  public void release(){
    climbControllerGroup.setInverted(true);
    climbControllerGroup.set(1);
  }

  public void stop(){
    climbControllerGroup.stopMotor();
  }
}
