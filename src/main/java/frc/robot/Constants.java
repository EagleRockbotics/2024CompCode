// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation2d;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static String APRILTAG_JSON_NAME = "tags.json";
  public static String CAMERA_JSON_NAME = "cams.json";
  public static String APRILTAG_POSITION_JSON_NAME = "tagPos.json";

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ElevatorConstants {
    public static int kRightElevatorCanID = 19;
    public static int kLeftElevatorCanID = 17;
    public static int kRightElevatorEncoderCanId = 16;
    public static int kLeftElevatorEncoderCanId = 21;
    public static double kElevatorMaxHeight = 0;
    public static double kElevatorDownHeight = 7.9;
    public static String kElevatorIdle = "idle";
    public static String kRelease = "max";
    public static String kReturn = "min";
    public static String kElevatorManual = "manual";
  }


  public static class ShooterIntakeConstants {
    public static Rotation2d kIntakeLowerPosition = Rotation2d.fromRotations(.4);   // in degrees
    public static Rotation2d kIntakeHigherPosition = Rotation2d.fromRotations(0.07);  // in degrees
    public static Rotation2d kShooterIntakePosition = Rotation2d.fromDegrees(-40);  // in degrees
    public static Rotation2d kShooterMaxPosition = Rotation2d.fromDegrees(80);
    public static double kShooterVelocity = 5;  // in meters per second
  }

  public static class IntakeConstants {
    public static int kPitchMotorCanId = 28;
    public static int kIntakeMotorCanId = 18;
    public static int kPitchEncoderCanId = 20;
    public static int kLimitSwitchPort = 0;
    public static double kPitchP = 0.1;
    public static double kPitchI = 0;
    public static double kPitchD = 0; // .1
    public static double kPitchKG = -.1;
    public static double kIntakeTolerance = 5 * Math.PI / 180;
  }

  public static class ShooterConstants {
    public static double kShootingRotatingP = 0.001;
    public static double kShootingRotatingI = 0;
    public static double kShootingRotatingD = 0;
    public static double kShootingKG = 0.07; // power needed to keep motor steay when facing straight out
    public static int kShootingRotatingCanId = 21;
    public static int kShooterIntakeCanId = 10;
    public static int kShootingLeftCanId = 12;
    public static int kShootingRightCanId = 13;
    public static int kLimitSwitchPort = 1;
    // public static double kShooterTolerance = 2 * Math.PI / 180; // in radians
    public static int kShooterRotatingEncoderCanId = 15;
    public static double kShooterTolerance = 2 * Math.PI / 180;
  }

  public static class FieldConstants {
    // constants here should be in meters
    public static double kSpeakerHeightClose = 2.11;
    public static double kSpeakerHeightFar = 1.98;
    public static double kCloseEdgeFromTag = .46;

    public static double kRedSpeakerTagID = 4;
    public static double kBlueSpeakerTagID = 7;
  }

  public static class RobotConstants {
    // constants here should be in meters
    public static double kShooterHeight = 4;
  }

  public static class ControllerConstants {
    public static int kDrivingJoystickPort = 0;
    public static int kDrivingJoystickX = 1;
    public static int kDrivingJoystickY = 0;
    public static int kDrivingJoystickZ = 2;
    public static double kDrivingSpeed = 1; //.75
    public static double kSteerSpeed = 1;
    public static double kSteerDeadzone = 0.1;
    public static double kDriveDeadzone = 0.1;
    public static int kHelperJoystickPort = 1;
  }

  public static class DriveConstants {
    public static double kMaxSpeedMetersPerSecond = 4.5;

    // Chassis configuration
    public static double kTrackWidth = Units.inchesToMeters(28);
    // Distance between centers of right and left wheels
    public static double kWheelBase = Units.inchesToMeters(28);
    // Distance between front and back wheels on robot
    public static SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
      );
    
    // SPARK MAX CAN IDs
    public static int kFrontLeftDrivingCanId = 6;
    public static int kRearLeftDrivingCanId = 4;
    public static int kFrontRightDrivingCanId = 8;
    public static int kRearRightDrivingCanId = 2;


    public static int kFrontLeftTurningCanId = 5;
    public static int kRearLeftTurningCanId = 3;
    public static int kFrontRightTurningCanId = 7;  
    public static int kRearRightTurningCanId = 1;


    public static int kFrontLeftEncoderCanId = 13; 
    public static int kRearLeftEncoderCanId = 10;
    public static int kFrontRightEncoderCanId = 12;
    public static int kRearRightEncoderCanId = 11; 

    public static int kPigeonGyro = 14;

    public static boolean kGyroReversed = false;

    //Pathplanner Translation PID Constants
    public static double TkP = 0.01;
    public static double TkI = 0.0;
    public static double TkD = 0.0;

    //Pathplanner Rotation PID Constants
    public static double RkP = 0.0001;
    public static double RkI = 0.0;
    public static double RkD = 0.0;

    //PID constants for controlling rotation of robot
    public static double kAutoTargettingP = .001;
    public static double kAutoTargettingI = 0;
    public static double kAutoTargettingD = 0;
  }


  public static class ModuleConstants {
    public static double kFreeSpeedRpm = 5676;

    public static boolean FL_driveInverted = true;
    public static boolean FL_steerInverted = false;
 
    public static boolean FR_driveInverted = false;
    public static boolean FR_steerInverted = false;

    public static boolean RL_driveInverted = false;
    public static boolean RL_steerInverted = false;

    public static boolean RR_driveInverted = false;
    public static boolean RR_steerInverted = false;


    // public static double kDrivingMotorPinionTeeth = 14;


    public static boolean kTurningEncoderInverted = true;


    public static double kDrivingMotorFreeSpeedRps = kFreeSpeedRpm / 60;
    // public static double kWheelDiameterMeters = .1;
    public static double kWheelCircumferenceMeters = .314;


    // public static double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static double kGearWheelRatio = 1/6.75;
    public static double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) * kGearWheelRatio;


    public static double kDrivingEncoderPositionFactor = kWheelCircumferenceMeters * kGearWheelRatio;
    public static double kDrivingEncoderVelocityFactor = kWheelCircumferenceMeters * kGearWheelRatio * 60.0;


    public static double kTurningEncoderPositionPIDMinInput = 0.0;
    public static double kTurningEncoderPositionPIDMaxInput = 360.0;

    public static double kDeadzone = 0.2;


    public static double kDrivingP = 0.001;
    public static double kDrivingI = 0;
    public static double kDrivingD = 0;
    public static double kDrivingFF = 0; //1/kDriveWheelFreeSpeedRps;
    public static double kDrivingMinOutput = -1;
    public static double kDrivingMaxOutput = 1;


    public static double kTurningP = 0.008;
    public static double kTurningI = 0;
    public static double kTurningD = 0;
    //public static double kTurningFF = 0.1;
    public static double kTurningMinOutput = -1;
    public static double kTurningMaxOutput = 1;


    public static com.revrobotics.CANSparkBase.IdleMode kDrivingMotorIdleMode = com.revrobotics.CANSparkBase.IdleMode.kBrake;
    public static com.revrobotics.CANSparkBase.IdleMode kTurningMotorIdleMode = com.revrobotics.CANSparkBase.IdleMode.kBrake;


    public static int kDrivingMotorCurrentLimit = 50; // amps
    public static int kTurningMotorCurrentLimit = 10;
  }
}


