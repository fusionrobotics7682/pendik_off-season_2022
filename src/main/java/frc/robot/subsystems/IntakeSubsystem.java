// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class IntakeSubsystem extends SubsystemBase {

  Victor intake1 = new Victor(Constants.IntakeConstants.INTAKE_1_PIN);
  Victor intake2 = new Victor(Constants.IntakeConstants.INTAKE_2_PIN);

  Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  DoubleSolenoid solenoids = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void getIn(){
    solenoids.set(Value.kForward);
    intake1.set(1);
    intake2.set(1);
  }

  public void getOut(){
    solenoids.set(Value.kForward);
    intake1.set(-1);
    intake2.set(-1);
  }

  public void stop(){
    solenoids.set(Value.kOff);
    intake1.set(0);
    intake2.set(0);
  }
}
