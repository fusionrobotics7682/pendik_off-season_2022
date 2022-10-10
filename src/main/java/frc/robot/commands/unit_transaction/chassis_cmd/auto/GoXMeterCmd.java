// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.unit_transaction.chassis_cmd.auto;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.triggers.PIDTrigger;

public class GoXMeterCmd extends CommandBase {
  
  DriveSubsystem driveSubsystem;
  double degreesSetpoint;
  double meterSetpoint;
  boolean isForward;
  PIDTrigger degreeTrigger;
  PIDTrigger meterTrigger;

  /** Creates a new TurnXDegrees. */
  public GoXMeterCmd(DriveSubsystem driveSubsystem, double meterSetpoint, double degreesSetpoint, PIDTrigger degreeTrigger, PIDTrigger meterTrigger, boolean isForward) {
    this.driveSubsystem = driveSubsystem;
    this.degreeTrigger = degreeTrigger;
    this.meterTrigger = meterTrigger;
    this.degreesSetpoint = degreesSetpoint;
    this.meterSetpoint = meterSetpoint;
    this.isForward = isForward;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!isForward){
      meterSetpoint *= -1;
    }
    driveSubsystem.basicAutoDrive(meterSetpoint, degreesSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double meterTriggerValue = meterTrigger.checkPidTrigger(driveSubsystem.getAverageEncoderDistance());
    double degreeTriggerValue = degreeTrigger.checkPidTrigger(driveSubsystem.getYaw());
    
    //return debouncer.calculate(driveSubsystem.getYaw() >= meterSetpoint-0.05 && driveSubsystem.getYaw() <= degreesSetpoint+0.05);
    if(degreeTriggerValue <= degreesSetpoint+3 && degreeTriggerValue >= degreesSetpoint-3 && 
       meterTriggerValue <= meterSetpoint+0.05 && meterTriggerValue >= meterSetpoint-0.05){
      return true;
    }
    return false;
  }
}