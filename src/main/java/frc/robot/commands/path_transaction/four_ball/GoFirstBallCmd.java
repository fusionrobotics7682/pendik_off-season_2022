// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.path_transaction.four_ball;

import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;

public class GoFirstBallCmd extends CommandBase {

  private DriveSubsystem driveSubsystem;
  private FeederSubsystem feederSubsystem;

  String goFirstBalltrajectoryPathJsonString = "Paths/four_ball/deploy/pathplanner/generatedJSON/go_first_ball.wpilib.json";
  Path goFirstBallTrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(goFirstBalltrajectoryPathJsonString);
  private Trajectory goFirstBallTrajectory = new Trajectory();  

  Timer timer = new Timer();

  /** Creates a new GoFirstBallCmd. */
  public GoFirstBallCmd(DriveSubsystem driveSubsystem, FeederSubsystem feederSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.feederSubsystem = feederSubsystem;
    addRequirements(driveSubsystem, feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      goFirstBallTrajectory = TrajectoryUtil.fromPathweaverJson(goFirstBallTrajectoryPath);
    }catch(IOException e){
      e.printStackTrace();
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
