// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.n_transaction;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.unit_transaction.chassis_cmd.GoXMeterCmd;
import frc.robot.commands.unit_transaction.intake_cmd.tele_op.GetIntakeCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThrowTwoBallScenarioCmd extends SequentialCommandGroup {
  /** Creates a new ThrowTwoBallScenarioCmd. */
  public ThrowTwoBallScenarioCmd(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelCommandGroup(new GoXMeterCmd(driveSubsystem, 0, 0.91, false), new GetIntakeCmd(intakeSubsystem)));
  }
}