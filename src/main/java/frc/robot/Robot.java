// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.concurrent.TimeUnit;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.Constants.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static SwerveDrive m_swerveDrive;
  private static Joystick m_DriveStick;
  private static Joystick m_HelperStick;

  private static SendableChooser<Command> autoChooser;
  private static Command m_autonomousCommand;

  private static VisionSubsystem m_visionSystem = new VisionSubsystem();

  private static Field2d limelightField = new Field2d();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_swerveDrive = new SwerveDrive();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
    m_DriveStick = new Joystick(ControllerConstants.kDrivingJoystickPort);
    m_HelperStick = new Joystick(ControllerConstants.kHelperJoystickPort);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = autoChooser.getSelected();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
   
  
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    //m_HelperStick.setRumble(RumbleType.kBothRumble, 1);
    m_swerveDrive.m_gyro.reset();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  /** This function is called periodically during operator control. */
  //Gyro angle was off by 90 degrees, sticks inverted incorrectly, and RobotRelative works better than field relative with no known downsides
  @Override
  public void teleopPeriodic() {
    var xDriveSpeed = -m_HelperStick.getRawAxis(0) * ControllerConstants.kDrivingSpeed;
    var yDriveSpeed = m_HelperStick.getRawAxis(1) * ControllerConstants.kDrivingSpeed;
    var rotDriveSpeed = -m_HelperStick.getRawAxis(4) * Math.PI * ControllerConstants.kSteerSpeed;

    if(m_HelperStick.getRawButtonReleased(5)) {
      m_swerveDrive.zeroHeading();
    }

    if (Math.abs(m_HelperStick.getRawAxis(4)) <= ControllerConstants.kSteerDeadzone) {
      rotDriveSpeed = 0;
    } 

    if (Math.abs(m_HelperStick.getRawAxis(1)) <= ControllerConstants.kDriveDeadzone) {
      yDriveSpeed = 0;
    }

    if (Math.abs(m_HelperStick.getRawAxis(0)) <= ControllerConstants.kDriveDeadzone) {
      xDriveSpeed = 0;
    }

    m_swerveDrive.driveFieldRelative(
        xDriveSpeed, 
        yDriveSpeed,
        rotDriveSpeed);

   
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    m_HelperStick.setRumble(RumbleType.kBothRumble, 0);
    if (m_autonomousCommand != null) { 
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    SmartDashboard.putData("Limelight Field", limelightField);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    m_visionSystem.periodic();
    if(m_DriveStick.getRawButtonReleased(10)) {
      m_swerveDrive.resetEncoders();
    }

    try { limelightField.setRobotPose(m_visionSystem.getRobotPose(Rotation2d.fromDegrees(0)).get(0)); } catch (Exception e) {}
    
    // double angle = (Math.atan2(m_DriveStick.getRawAxis(1), m_DriveStick.getRawAxis(0)) * 180 / Math.PI) - 90;
    // double speed = Math.sqrt(Math.pow(m_DriveStick.getRawAxis(1), 2) + Math.pow(m_DriveStick.getRawAxis(0), 2));
    // if (Math.abs(speed) > ModuleConstants.kDeadzone) {
    //   m_swerveDrive.PIDTuningHelper(angle, speed);
    // } else {
    //   m_swerveDrive.stopModules();
    // }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
