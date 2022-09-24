// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.n_transaction;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.unit_transaction.chassis_cmd.auto.GoXMeterCmd;
import frc.robot.commands.unit_transaction.feeder_cmd.auto.GetInXSecondFeederCmd;
import frc.robot.commands.unit_transaction.intake_cmd.auto.GetIntakeXSecondCmd;
import frc.robot.commands.unit_transaction.shooter_cmd.auto.StopShooterCmd;
import frc.robot.commands.unit_transaction.shooter_cmd.tele_op.ShootTarmacCloserTeleCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThrowTwoBallCmd extends SequentialCommandGroup {
  /** Creates a new ThrowTwoBallScenarioCmd. */
  public ThrowTwoBallCmd(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(new GoXMeterCmd(driveSubsystem, 0, 0.91, false), new GetIntakeXSecondCmd(intakeSubsystem, 3)), 
      new ParallelCommandGroup(new GoXMeterCmd(driveSubsystem, 0, 1.21, true), new GetInXSecondFeederCmd(feederSubsystem, 0.7), new ShootTarmacCloserTeleCmd(shooterSubsystem, 20)),
      new GetInXSecondFeederCmd(feederSubsystem, 1.5),
      new StopShooterCmd(shooterSubsystem));
  }
}