// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.unit_transaction.shooter_cmd;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCmd extends CommandBase {

  private ShooterSubsystem shooterSubsystem;
  private FeederSubsystem feederSubsystem;

  /** Creates a new ShootCmd. */
  public ShootCmd(ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.feederSubsystem = feederSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(DriverStation.getAlliance().name() == "BLUE"){
      if(feederSubsystem.getShooterColorSensorV3().getBlue() <= Constants.ShooterConstants.DEFAULT_BLUE_VALUE){
        shooterSubsystem.idleShot();
        feederSubsystem.getIn();
      } else{
        feederSubsystem.getIn();
        shooterSubsystem.shootTarmacCloser();
      }
    }
      else if(DriverStation.getAlliance().name() == "RED"){
        if(feederSubsystem.getShooterColorSensorV3().getRed() <= Constants.ShooterConstants.DEFAULT_RED_VALUE){
          shooterSubsystem.idleShot();
          feederSubsystem.getIn();
        }
        else{
          feederSubsystem.getIn();
          shooterSubsystem.shootTarmacCloser();
        }
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopMotors();
    feederSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
