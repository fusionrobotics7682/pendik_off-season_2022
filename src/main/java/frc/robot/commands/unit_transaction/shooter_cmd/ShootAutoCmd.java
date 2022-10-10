// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.unit_transaction.shooter_cmd;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAutoCmd extends CommandBase {
  private ShooterSubsystem shooterSubsystem;
  private FeederSubsystem feederSubsystem;

  /** Creates a new ShootCmd. */
  public ShootAutoCmd(ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem) {
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
    shooterSubsystem.shootTarmacCloser();
    if(shooterSubsystem.getCurrentVelocity() >= 20){
      feederSubsystem.getIn();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Run cmd until balls are shot
    if(feederSubsystem.getShooterColorSensorV3().getProximity() < 100 && feederSubsystem.getFeederColorSensorV3().getProximity() < 100){
      return true;
    }
    return false;
  }
}