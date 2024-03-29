// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.n_transaction.ThrowThreeBallCmd;
import frc.robot.commands.n_transaction.ThrowTwoBallCmd;
import frc.robot.commands.unit_transaction.chassis_cmd.tele_op.ArcadeDriveCmd;
import frc.robot.commands.unit_transaction.feeder_cmd.tele_op.GetInFeederCmd;
import frc.robot.commands.unit_transaction.feeder_cmd.tele_op.GetOutFeederCmd;
import frc.robot.commands.unit_transaction.intake_cmd.GetInCmd;
import frc.robot.commands.unit_transaction.intake_cmd.tele_op.GetIntakeCmd;
import frc.robot.commands.unit_transaction.intake_cmd.tele_op.GetOuttakeCmd;
import frc.robot.commands.unit_transaction.shooter_cmd.ShootCmd;
import frc.robot.commands.unit_transaction.shooter_cmd.tele_op.ShootTarmacCloserTeleCmd;
import frc.robot.constants.JoystickIOConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.triggers.PIDTrigger;
import frc.robot.triggers.TimerTrigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  PIDTrigger degreeTrigger = new PIDTrigger();
  PIDTrigger meterTrigger = new PIDTrigger();
  TimerTrigger timerTrigger = new TimerTrigger();

  // Cameras and joysticks defined here
  Joystick joystick = JoystickIOConstants.getJoystick();
  UsbCamera frontCamera;
  UsbCamera rearCamera;

  // The robot's subsystems and commands are defined here...
  DriveSubsystem driveSubsystem = new DriveSubsystem();
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  FeederSubsystem feederSubsystem = new FeederSubsystem();
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  ThrowTwoBallCmd twoBallCmd = new ThrowTwoBallCmd(driveSubsystem, intakeSubsystem, feederSubsystem, shooterSubsystem, degreeTrigger, meterTrigger);
  ThrowThreeBallCmd threeBallCmd = new ThrowThreeBallCmd(driveSubsystem, intakeSubsystem, feederSubsystem, shooterSubsystem, degreeTrigger, meterTrigger);

  private SendableChooser autoChooser = new SendableChooser();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveSubsystem.setDefaultCommand(new ArcadeDriveCmd(driveSubsystem));
    frontCamera = CameraServer.startAutomaticCapture(0);
    rearCamera = CameraServer.startAutomaticCapture(1);
    autoChooser.setDefaultOption("2 Ball Auto :", twoBallCmd);
    autoChooser.addOption("3 Ball Auto :", threeBallCmd);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(joystick, 2).whileActiveContinuous(new GetInCmd(intakeSubsystem, feederSubsystem));
    new JoystickButton(joystick, 3).whileActiveContinuous(new ShootCmd(shooterSubsystem, feederSubsystem));
    new JoystickButton(joystick, 4).whileActiveContinuous(new GetOuttakeCmd(intakeSubsystem));
    new JoystickButton(joystick, 5).whileActiveContinuous(new GetOutFeederCmd(feederSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
