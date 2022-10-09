// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.frc_auto_core.AutoConfigurer;
import frc.robot.frc_auto_core.CircularDrive;
import frc.robot.frc_auto_core.RotationalDrive;
import frc.robot.frc_auto_core.StraightDrive;
import frc.robot.constants.Constants;
import frc.robot.constants.JoystickIOConstants;
import frc.robot.constants.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  // Motors
  WPI_VictorSPX leftMotor1 = new WPI_VictorSPX(Constants.DriveConstants.LEFT_MOTOR_PIN_1);
  WPI_VictorSPX leftMotor2 = new WPI_VictorSPX(Constants.DriveConstants.LEFT_MOTOR_PIN_2);
  WPI_VictorSPX leftMotor3 = new WPI_VictorSPX(Constants.DriveConstants.LEFT_MOTOR_PIN_3);

  WPI_VictorSPX rightMotor1 = new WPI_VictorSPX(Constants.DriveConstants.RIGHT_MOTOR_PIN_1);
  WPI_VictorSPX rightMotor2 = new WPI_VictorSPX(Constants.DriveConstants.RIGHT_MOTOR_PIN_2);
  WPI_VictorSPX rightMotor3 = new WPI_VictorSPX(Constants.DriveConstants.RIGHT_MOTOR_PIN_3);

  // The robot's drive
  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMotor1, rightMotor1);

  // The Encoders drive encoder
  private final Encoder leftEncoder = new Encoder(Constants.DriveConstants.kLeftEncoderPorts[0], Constants.DriveConstants.kLeftEncoderPorts[1]);
  private final Encoder rightEncoder = new Encoder(Constants.DriveConstants.kRightEncoderPorts[0], Constants.DriveConstants.kRightEncoderPorts[1], true);

  // The Navx sensor
  private final AHRS navx = new AHRS(Port.kMXP);

  // The FRC Auto-Core drive implementations
  AutoConfigurer configurer = new AutoConfigurer();
  RotationalDrive rotationalDrive = new RotationalDrive(configurer, navx);
  StraightDrive straightDrive = new StraightDrive(configurer, leftEncoder, rightEncoder);
  CircularDrive circularDrive = new CircularDrive(configurer, rightEncoder, leftEncoder);
  
  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

/** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
  // We need to invert one side of the drivetrain so that positive voltages
  // result in both sides moving forward. Depending on how your robot's
  // gearbox is constructed, you might have to invert the left side instead.
  rightMotor1.setInverted(true);
  rightMotor2.follow(rightMotor1);
  rightMotor3.follow(rightMotor1);
  
  leftMotor2.follow(leftMotor1);
  leftMotor3.follow(leftMotor1);

  // Sets the distance per pulse for the encoders
  leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
  rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

  resetSensors();
  differentialDrive.setMaxOutput(Constants.DriveConstants.ROBOT_MAX_SPEED);
  m_odometry = new DifferentialDriveOdometry(navx.getRotation2d());
  }

@Override
public void periodic() {
 // Update the odometry in the periodic block
 m_odometry.update(
  navx.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
}

public void arcadeDrive(){
}

public void tankDrive(){
  differentialDrive.tankDrive(JoystickIOConstants.getRawAxis(1), JoystickIOConstants.getRawAxis(5));
}

// For encoder drive
public void basicAutoDrive(double meterSetpoint, double degreeSetpoint){
  differentialDrive.arcadeDrive(straightDrive.goXmeter(meterSetpoint), rotationalDrive.turnXDegrees(degreeSetpoint));
}

// For encoder&gyro drive
/*public void advancedAutoDrive(double degreesSetpoint, double meterSetpoint){
  double [] outputs = circularDrive.calculateArcSetpoints(degreesSetpoint, meterSetpoint);
  differentialDrive.tankDrive(outputs[0], outputs[1]);
}*/

// For Only Gyro drive
public void gyroAutoDrive(double degreesSetpoint){
  differentialDrive.arcadeDrive(0, rotationalDrive.turnXDegrees(degreesSetpoint));
}

/**
* Returns the currently-estimated pose of the robot.
*
* @return The pose.
*/
public Pose2d getPose() {
 return m_odometry.getPoseMeters();
}

/**
* Returns the current wheel speeds of the robot.
*
* @return The current wheel speeds.
*/
public DifferentialDriveWheelSpeeds getWheelSpeeds() {
 return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
}

/**
* Resets the odometry to the specified pose.
*
* @param pose The pose to which to set the odometry.
*/
public void resetOdometry(Pose2d pose) {
 resetEncoders();
 m_odometry.resetPosition(pose, navx.getRotation2d());
}

/**
* Controls the left and right sides of the drive directly with voltages.
*
* @param leftVolts the commanded left output
* @param rightVolts the commanded right output
*/
public void tankDriveVolts(double leftVolts, double rightVolts) {
 leftMotor1.setVoltage(leftVolts);
 rightMotor1.setVoltage(rightVolts);
 differentialDrive.feed();
}

public void resetSensors(){
  resetEncoders();
  resetNavX();
}

/** Resets the drive encoders to currently read a position of 0. */
public void resetEncoders() {
  leftEncoder.reset();
  rightEncoder.reset();
}

public void resetNavX(){
  navx.reset();
}

public void stopDrive(){
  differentialDrive.stopMotor();
}

/**
* Gets the average distance of the two encoders.
*
* @return the average of the two encoder readings
*/
public double getAverageEncoderDistance() {
 return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0 * Constants.DriveConstants.K_DRIVE_TICK_2_FEET;
}

/**
* Gets the left drive encoder.
*
* @return the left drive encoder
*/
public Encoder getLeftEncoder() {
 return leftEncoder;
}

/**
* Gets the right drive encoder.
*
* @return the right drive encoder
*/
public Encoder getRightEncoder() {
 return rightEncoder;
}

/**
* Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
*
* @param maxOutput the maximum output to which the drive will be constrained
*/
public void setMaxOutput(double maxOutput) {
 differentialDrive.setMaxOutput(maxOutput);
}

/** Zeroes the heading of the robot. */
public void zeroHeading() {
  navx.reset();
}

public double getYaw(){
  return navx.getYaw();
}

/**
* Returns the heading of the robot.
*
* @return the robot's heading in degrees, from -180 to 180
*/
public double getHeading() {
 return navx.getRotation2d().getDegrees();
}

/**
* Returns the turn rate of the robot.
*
* @return The turn rate of the robot, in degrees per second
*/
public double getTurnRate() {
 return -navx.getRate();
}
}
