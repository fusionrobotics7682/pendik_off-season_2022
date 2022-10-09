// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class FeederSubsystem extends SubsystemBase {

  Victor feeder1 = new Victor(Constants.FeederConstants.FEEDER_1_PIN);
  Victor feeder2 = new Victor(Constants.FeederConstants.FEEDER_2_PIN);

  private final ColorSensorV3 feederColorSensorV3 = new ColorSensorV3(Constants.FeederConstants.I2C_ROBORIO);
  private final ColorSensorV3 shooterColorSensorV3 = new ColorSensorV3(Constants.FeederConstants.I2C_ROBORIO);

  MotorControllerGroup feederControllerGroup = new MotorControllerGroup(feeder1, feeder2);

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void getIn(){
    feederControllerGroup.set(1);
  }

  public void getOut(){
    feederControllerGroup.setInverted(true);
    feederControllerGroup.set(1);
  }
  
  public void stop(){
    feederControllerGroup.stopMotor();
  }

  public ColorSensorV3 getFeederColorSensorV3(){
    return feederColorSensorV3;
  }

  public ColorSensorV3 getShooterColorSensorV3(){
    return shooterColorSensorV3;
  }
}
