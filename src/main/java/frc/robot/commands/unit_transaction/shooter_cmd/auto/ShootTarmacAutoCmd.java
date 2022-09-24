// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.unit_transaction.shooter_cmd.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootTarmacAutoCmd extends CommandBase {

  ShooterSubsystem shooterSubsystem;
  Timer timer = new Timer();

  double setPointVelocity;

  /** Creates a new ShootTarmacCommand. */
  public ShootTarmacAutoCmd(ShooterSubsystem shooterSubsystem, double setPointVelocity) {
    this.shooterSubsystem = shooterSubsystem;
    this.setPointVelocity = setPointVelocity;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.shoot(setPointVelocity, timer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
