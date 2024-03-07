// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.*;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.PowerDistribution;
import java.util.ArrayList;

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

  private static Elevator m_elevator;

  // private static SendableChooser<Command> autoChooser;
  private static Command m_autonomousCommand;

  private static VisionSubsystem m_visionSystem = new VisionSubsystem();

  private static Field2d limelightField = new Field2d();

  private static PowerDistribution PD;
  private ArrayList<Double> voltage = new ArrayList<Double>();


  // autonomous
  private SendableChooser<String> joshAutoChooser;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    joshAutoChooser = new SendableChooser<>();
    joshAutoChooser.addOption("empty", "empty");
    joshAutoChooser.addOption("Drive Forward 6 Meters", "driveFwd");
    joshAutoChooser.setDefaultOption("Drive Forward", "driveFwd");
    // joshAutoChooser.initSendable(null);
    SmartDashboard.putData("Auto Chooser", joshAutoChooser);
    m_swerveDrive = new SwerveDrive();
    // autoChooser = AutoBuilder.buildAutoChooser();
    //  SmartDashboard.putData(autoChooser);
    m_DriveStick = new Joystick(ControllerConstants.kDrivingJoystickPort);
    m_HelperStick = new Joystick(ControllerConstants.kHelperJoystickPort);
    m_elevator = new Elevator();
    CameraServer.startAutomaticCapture(0);

    m_swerveDrive.m_gyro.reset();

    PD = new PowerDistribution(1, ModuleType.kRev);
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
    double v = PD.getVoltage();
    voltage.add(v);
    SmartDashboard.putNumber("voltage", v);
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
    m_swerveDrive.zeroHeading();
    m_swerveDrive.resetPose(new Pose2d());
    // a_timer1.reset();
    // a_timer1.start();

    // m_autonomousCommand = autoChooser.getSelected();

    // if (m_autonomousCommand != null) {
    //   // m_autonomousCommand.schedule();
    // }
  }

  /** This function is called periodically during autonomous. */
  //8.28-1.55
  @Override
  public void autonomousPeriodic() {
    if (joshAutoChooser.getSelected() == "empty") {
      m_swerveDrive.driveFieldRelative(0, 0, 0);
    } else if (joshAutoChooser.getSelected() == "driveFwd") {
      if (Math.abs(m_swerveDrive.getPose().getX()) <= (3)) { //6 meters, actually 7.5; 3 meters, actually 5.96 meters
        SmartDashboard.putNumber("x Position", m_swerveDrive.getPose().getX());
        m_swerveDrive.driveFieldRelative(0, -.1, 0);
      } else {
        m_swerveDrive.driveFieldRelative(0, 0, 0);
      }
    }
    
    m_elevator.idle();
    // if (a_timer1.get() < 2.6) {
    //   m_swerveDrive.driveFieldRelative(0, .5, 0);
    //   SmartDashboard.putNumber("time", a_timer1.get());
    // } else {
    //   m_swerveDrive.driveFieldRelative(0, 0, 0);
    // }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_elevator.teleopInit();
  }

  /** This function is called periodically during operator control. */
  //Gyro angle was off by 90 degrees, sticks inverted incorrectly, and RobotRelative works better than field relative with no known downsides
  @Override
  public void teleopPeriodic() {
    m_elevator.manual(m_HelperStick.getRawAxis(2), m_HelperStick.getRawAxis(3));

    double[] driveValues = m_swerveDrive.JoystickConverter(m_DriveStick.getRawAxis(0), m_DriveStick.getRawAxis(1), m_DriveStick.getRawAxis(4));
    
    if(m_HelperStick.getRawButtonReleased(3)) {
      m_swerveDrive.zeroHeading();;
    }

    m_swerveDrive.driveFieldRelative(
        driveValues[0], 
        driveValues[1],
        driveValues[2]);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    m_DriveStick.setRumble(RumbleType.kBothRumble, 0);
    if (m_autonomousCommand != null) { 
      m_autonomousCommand.cancel();
    }
    System.out.println(voltage);
    SmartDashboard.putString("voltage array", voltage.toString());
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    SmartDashboard.putData("Limelight Field", limelightField);
    m_elevator.teleopInit(); 
    // m_elevator.changeCommand(ElevatorConstants.kElevatorManual);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    m_visionSystem.periodic();
    if(m_DriveStick.getRawButtonReleased(10)) {
      m_swerveDrive.resetEncoders();
    }

    try { limelightField.setRobotPose(m_visionSystem.getRobotPose(Rotation2d.fromDegrees(0)).get(0)); } catch (Exception e) {}
    
    // m_elevator.manual(m_HelperStick.getRawAxis(2), m_HelperStick.getRawAxis(3));
    SmartDashboard.putNumber("Right Elevator Position", m_elevator.getPositions()[0]);
    SmartDashboard.putNumber("Left Elevator Position", m_elevator.getPositions()[1]);

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
