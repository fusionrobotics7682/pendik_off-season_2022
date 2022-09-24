// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.frc_auto_core.AutoConfigurer;
import frc.robot.frc_auto_core.VelocityController;
import frc.robot.constants.Constants;

public class ShooterSubsystem extends SubsystemBase {

  CANSparkMax shooter1 = new CANSparkMax(Constants.ShooterConstants.SHOOTER_1_PIN, MotorType.kBrushless);
  CANSparkMax shooter2 = new CANSparkMax(Constants.ShooterConstants.SHOOTER_2_PIN, MotorType.kBrushless);

  AutoConfigurer configurer = new AutoConfigurer();
  VelocityController controller = new VelocityController(configurer, shooter1.getEncoder());
  
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot(double setPointVelocity, double currentTime){
    double currentVelocity = controller.runXVelocity(setPointVelocity, currentTime);
    shooter1.set(currentVelocity);
    shooter2.follow(shooter1);
  }
  
  public void shootTarmacCloser(double currentTime){
    shoot(22, currentTime);
  }

  public void shootTarmacFarther(double currentTime){
    shoot(24.6, currentTime);
  }

  public void stopMotors(){
    shooter1.stopMotor();
    shooter2.stopMotor();
  }

}
