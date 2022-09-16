// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.auto_core.AutoConfigurer;

public class ShooterSubsystem extends SubsystemBase {

  CANSparkMax shooter1 = new CANSparkMax(Constants.ShooterConstants.SHOOTER_1_PIN, MotorType.kBrushless);
  CANSparkMax shooter2 = new CANSparkMax(Constants.ShooterConstants.SHOOTER_2_PIN, MotorType.kBrushless);

  AutoConfigurer configurer = new AutoConfigurer();
  
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot(double setPointRPM){
    // RPM
    double currentRPM = shooter1.getEncoder().getVelocity();

    shooter1.set(configurer.getPIDController(Constants.ShooterConstants.KP, Constants.ShooterConstants.KI, Constants.ShooterConstants.KD).calculate(currentRPM, setPointRPM));
    shooter2.follow(shooter1);
  }
  
  public void shootInTarmac(double setPointRPM){
    shoot(setPointRPM);
  }

  public void shootLaunchpad(double setPointRPM){
    shoot(setPointRPM);
  }

}
