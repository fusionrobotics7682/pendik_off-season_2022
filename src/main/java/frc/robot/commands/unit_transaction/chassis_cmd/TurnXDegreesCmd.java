// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.unit_transaction.chassis_cmd;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnXDegreesCmd extends CommandBase {

  DriveSubsystem driveSubsystem;
  double degreesSetpoint;
  Debouncer debouncer = new Debouncer(0.5);

  /** Creates a new TurnXDegrees. */
  public TurnXDegreesCmd(DriveSubsystem driveSubsystem, double degreesSetpoint) {
    this.driveSubsystem = driveSubsystem;
    this.degreesSetpoint = degreesSetpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.resetSensors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.gyroAutoDrive(degreesSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return checkPidStatus();
  }

  public boolean checkPidStatus(){
    return debouncer.calculate(driveSubsystem.getYaw() >= degreesSetpoint-3 && driveSubsystem.getYaw() <= degreesSetpoint+3);
  }
}
/*
  if(driveSubsystem.getYaw() >= degreesSetpoint-3 && driveSubsystem.getYaw() <= degreesSetpoint+3){
      timer.start();
      // Last condition is protecting the pre-firection
      if(timer.get() <= 1.5 && timer.get() >= 0.8){
        return true;
      }
*/ 
