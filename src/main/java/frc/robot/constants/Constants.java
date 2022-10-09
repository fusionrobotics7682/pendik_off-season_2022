// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int LEFT_MOTOR_PIN_1 = 3;
    public static final int LEFT_MOTOR_PIN_2 = 4;
    public static final int LEFT_MOTOR_PIN_3 = 5;
    public static final int RIGHT_MOTOR_PIN_1 = 0;
    public static final int RIGHT_MOTOR_PIN_2 = 1;
    public static final int RIGHT_MOTOR_PIN_3 = 2;

    public static final double ROBOT_MAX_SPEED = 0.3;

    public static final int[] kLeftEncoderPorts = new int[] {0, 1};
    public static final int[] kRightEncoderPorts = new int[] {2, 3};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final double ENCODER_KP = 0.5;
    public static final double ENCODER_KI = 0;
    public static final double ENCODER_KD = 0;

    public static final double GYRO_KP = 0.5;
    public static final double GYRO_KI = 0;
    public static final double GYRO_KD = 0;

    public static final double REDUCTION_GEAR = 10.71;
    public static final double WHEEL_DIAMETER_INCHES = 6;
    public static final double ENCODER_PPR = 80;

    public static final double K_DRIVE_TICK_2_FEET = REDUCTION_GEAR / Math.PI * WHEEL_DIAMETER_INCHES * ENCODER_PPR / 12;

    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 80;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;
  }

  public static final class IntakeConstants{
    public static final int INTAKE_1_PIN = 7;
    public static final int INTAKE_2_PIN = 8;
  }

  public static final class FeederConstants{
    public static final int FEEDER_1_PIN = 9;
    public static final int FEEDER_2_PIN = 10;

    public static final Port I2C_ROBORIO = Port.kOnboard;
    public static final Port I2C_NAVX = Port.kMXP;
  }

  public static final class ShooterConstants{
    public static final int SHOOTER_1_PIN = 11;
    public static final int SHOOTER_2_PIN = 12;

    public static final double REDUCTION_GEAR = 2;
    public static final double WHEEL_DIAMETER_INCHES = 5.51;
    public static final double ENCODER_PPR = 42;

    public static final double K_DRIVE_TICK_2_FEET = REDUCTION_GEAR / Math.PI * WHEEL_DIAMETER_INCHES * ENCODER_PPR / 12;
    
    public static final double KP = 0.5;
    public static final double KI = 0;
    public static final double KD = 0;
    
    public static final double DEFAULT_BLUE_VALUE = 0.3;
    public static final double DEFAULT_RED_VALUE = 0.4;
  }

  public static final class ClimbConstants{
    public static final int CLIMB_PIN_1 = 11;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}
