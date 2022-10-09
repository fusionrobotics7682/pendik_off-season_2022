// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.path_transaction.four_ball;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.unit_transaction.feeder_cmd.auto.GetInXSecondFeederCmd;
import frc.robot.commands.unit_transaction.feeder_cmd.tele_op.GetInFeederCmd;
import frc.robot.commands.unit_transaction.intake_cmd.auto.GetIntakeXSecondCmd;
import frc.robot.commands.unit_transaction.intake_cmd.tele_op.GetIntakeCmd;
import frc.robot.commands.unit_transaction.shooter_cmd.auto.ShootTarmacAutoCmd;
import frc.robot.constants.Constants.AutoConstants;
import frc.robot.constants.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SequentialFourBallCmd extends SequentialCommandGroup {

  private DriveSubsystem driveSubsystem;
  private FeederSubsystem feederSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private Timer timer;

  /* General Path of the PathPlanner JSON Files */
  String GENERAL_PATH = "Paths/four_ball/deploy/pathplanner/generatedJSON";

  /* First Ball File Names */ 
  String GO_FIRST_BALL_FILE_PATH = "go_first_ball.wpilib.json";
  String GO_FIRST_POSITION_FILE_PATH = "go_first_positon.wpilib.json";
  String GO_SECOND_BALL_FILE_PATH = "go_second_ball.wpilib.json";
  String GO_SECOND_POSITION_FILE_PATH = "go_second_position.wpilib.json";

  String goFirstBalltrajectoryPathJsonString = GENERAL_PATH + GO_FIRST_BALL_FILE_PATH;
  Path goFirstBallTrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(goFirstBalltrajectoryPathJsonString);
  private Trajectory goFirstBallTrajectory = new Trajectory();  

  String goFirstPositiontrajectoryPathJsonString = GENERAL_PATH + GO_FIRST_POSITION_FILE_PATH;
  Path goFirstPositionTrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(goFirstPositiontrajectoryPathJsonString);
  private Trajectory goFirstPositionTrajectory = new Trajectory();  

  String goSecondBalltrajectoryPathJsonString = GENERAL_PATH + GO_SECOND_BALL_FILE_PATH;
  Path goSecondBallTrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(goFirstBalltrajectoryPathJsonString);
  private Trajectory goSecondBallTrajectory = new Trajectory();  

  String goSecondPositiontrajectoryPathJsonString = GENERAL_PATH + GO_SECOND_POSITION_FILE_PATH;
  Path goSecondPositionTrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(goSecondPositiontrajectoryPathJsonString);
  private Trajectory goSecondPositionTrajectory = new Trajectory(); 
  

  /** Creates a new SequentialFourBallCmd. */
  public SequentialFourBallCmd(DriveSubsystem driveSubsystem) {  
    this.driveSubsystem = driveSubsystem;
    timer = new Timer();
    try {
      goFirstBallTrajectory = TrajectoryUtil.fromPathweaverJson(goFirstBallTrajectoryPath);
      goFirstPositionTrajectory = TrajectoryUtil.fromPathweaverJson(goFirstPositionTrajectoryPath);
      goSecondBallTrajectory = TrajectoryUtil.fromPathweaverJson(goSecondBallTrajectoryPath);
      goSecondPositionTrajectory = TrajectoryUtil.fromPathweaverJson(goSecondPositionTrajectoryPath);
    } catch (IOException e) {
      e.printStackTrace();
    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelCommandGroup(
      new GetIntakeXSecondCmd(intakeSubsystem, 1.8), GoFirstBallCommand(), new ShootTarmacAutoCmd(shooterSubsystem, 15)),
      new GetInXSecondFeederCmd(feederSubsystem, 1.5), new ShootTarmacAutoCmd(shooterSubsystem, 22),
      new ParallelCommandGroup(
        new ShootTarmacAutoCmd(shooterSubsystem, 12),
        GoSecondBallCommand(),
        new GetIntakeXSecondCmd(intakeSubsystem, 3.5)),
      new ParallelCommandGroup(
        GoSecondPositionCommand(),
        new ShootTarmacAutoCmd(shooterSubsystem, 22)
      ),
      new GetInXSecondFeederCmd(feederSubsystem, 3.5));
  }

  public Command GoFirstBallCommand(){
    return new RamseteCommand(
        goFirstBallTrajectory,
        driveSubsystem::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        driveSubsystem::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        driveSubsystem::tankDriveVolts,
        driveSubsystem);
  }

  public Command GoFirstPositionCommand(){
    timer.delay(1);
    return new RamseteCommand(
        goFirstPositionTrajectory,
        driveSubsystem::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        driveSubsystem::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        driveSubsystem::tankDriveVolts,
        driveSubsystem);
  }

  public Command GoSecondBallCommand(){
    timer.delay(1);
    return new RamseteCommand(
        goSecondBallTrajectory,
        driveSubsystem::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        driveSubsystem::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        driveSubsystem::tankDriveVolts,
        driveSubsystem);
  }

  public Command GoSecondPositionCommand(){
    timer.delay(1);
    return new RamseteCommand(
        goSecondPositionTrajectory,
        driveSubsystem::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        driveSubsystem::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        driveSubsystem::tankDriveVolts,
        driveSubsystem);
  }
}