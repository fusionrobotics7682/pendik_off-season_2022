// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.unit_transaction.feeder_cmd.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;

public class GetInXSecondFeederCmd extends CommandBase {
  /** Creates a new GetInXSecondFeederCmd. */
  FeederSubsystem feederSubsytem;
  Timer timer = new Timer();

  double secondSetpoint;

  /** Creates a new GetInXSecondCmd. */
  public GetInXSecondFeederCmd(FeederSubsystem feederSubsytem, double secondSetpoint) {
    this.feederSubsytem = feederSubsytem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feederSubsytem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feederSubsytem.getIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feederSubsytem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.get() <= secondSetpoint){
      return false;
    }
    return true;
  }
}
