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
import edu.wpi.first.cameraserver.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;

import java.util.ArrayList;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;

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

  private static Shooter m_shooter;
  private static Intake m_intake;


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

    m_shooter = new Shooter();
    m_intake = new Intake();

    m_swerveDrive.m_gyro.reset();
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
    SmartDashboard.putNumberArray("Limelight Distance Vector Magnitude", m_visionSystem.getDistanceMagnitudesList());
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
   *---*999
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
    step_num = 0;
    step_num2 = 0;
    m_shooter.resetAngle();
    // a_timer1.reset();
    // a_timer1.start();

    // m_autonomousCommand = autoChooser.getSelected();

    // if (m_autonomousCommand != null) {
    //   // m_autonomousCommand.schedule();
    // }
  }
  double step_num = 0;
  double step_num2 = 0;
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("Step Num", step_num2);
    SmartDashboard.putNumber("swerve position", Math.abs(m_swerveDrive.getPose().getX()));
    step_num += .02;
    if (joshAutoChooser.getSelected() == "empty") {
      m_swerveDrive.driveFieldRelative(0, 0, 0);
    } else if (joshAutoChooser.getSelected() == "driveFwd") {
      if (step_num < 6) {
        m_shooter.runShooter(-1);
        m_shooter.runIntake(-1);
        m_intake.runIntake(0);
        m_intake.setPosition(false);
      } else if (step_num < 8) {
        m_shooter.runShooter(-1);
        m_shooter.runIntake(-1);
        m_intake.runIntake(-1);
        m_intake.setPosition(false);
      } else {
        // if (Math.abs(m_swerveDrive.getPose().getX()) > (6)) {
        if (step_num < 15) {
          m_swerveDrive.driveFieldRelative(0, -.5, 0);
        } else {
          m_swerveDrive.driveFieldRelative(0, 0, 0);
        }
        m_shooter.runShooter(0);
        m_shooter.runIntake(0);
        m_intake.runIntake(0);
        m_intake.setPosition(false);
      }
    }
    
    // m_elevator.idle();
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
    // m_elevator.teleopInit();
  }

  /** This function is called periodically during operator control. */
  //Gyro angle was off by 90 degrees, sticks inverted incorrectly, and RobotRelative works better than field relative with no known downsides
  @Override
  public void teleopPeriodic() {
    m_elevator.manual(m_HelperStick.getRawAxis(3) * (m_HelperStick.getRawButton(6) ? 1d : -1d), m_HelperStick.getRawAxis(2) * (m_HelperStick.getRawButton(5) ? 1d : -1d));

    double[] driveValues = m_swerveDrive.JoystickConverter(m_DriveStick.getRawAxis(0), m_DriveStick.getRawAxis(1), m_DriveStick.getRawAxis(4));
    
    if(m_HelperStick.getRawButtonReleased(8)) {
      m_swerveDrive.zeroHeading();
    }

    m_swerveDrive.driveFieldRelative(
        driveValues[0], 
        driveValues[1],
        driveValues[2]
    );

    // m_swerveDrive.driveRobotRelative(
    //     driveValues[0], 
    //     driveValues[1],
    //     driveValues[2]
    // );

    // SmartDashboard.putNumberArray("Limelight Distance Vector Magnitude", (Double[]) m_visionSystem.getDistancesMagnitudes().values().toArray());
    if (m_HelperStick.getRawButton(3)) {
      m_intake.setPosition(true);
    } else {
      m_intake.setPosition(false);
      // m_intake.runIntake(0);
    }
    if (m_HelperStick.getRawButton(1)) {
      m_intake.runIntake(.5);
    } else {
      m_intake.runIntake(0);
    }
    // if (m_HelperStick.getRawButton(2)) {
    //   m_shooter.runIntake(-1);
    // } else {
    //   m_shooter.runIntake(0);
    // }
    m_shooter.runIntake(0);
    if (m_HelperStick.getRawButton(4)) {
      m_intake.runIntake(-1);
      m_shooter.runIntake(-1);
    }
    if (m_HelperStick.getRawButton(2)) {
      m_shooter.runShooter(-1);
      m_shooter.runIntake(-1);
    } else {
      m_shooter.runShooter(0);
    }

    if (m_HelperStick.getRawButtonReleased(7)){
      if(ControllerConstants.kDrivingSpeed == 1){
        ControllerConstants.kDrivingSpeed = 0.15;
        ControllerConstants.kSteerSpeed = 0.1;
      } else {
        ControllerConstants.kDrivingSpeed = 1;
        ControllerConstants.kSteerSpeed = 0.75;
      }
    }
    // if (m_HelperStick.getRawButton(3)) {
    //   m_intake.runIntake(0.3);
    // } else if(m_HelperStick.getRawButton(4)){
    //   m_intake.runIntake(-0.3);
    // } else {
    //   m_intake.runIntake(0);
    // }
    //m_shooter.runRotatingMotor(m_HelperStick.getRawAxis(1)*.4);
    if (m_HelperStick.getRawButton(7)) {
      m_intake.runPitchMotor(m_HelperStick.getRawAxis(5)*.4); //(m_HelperStick.getRawAxis(2)-m_HelperStick.getRawAxis(3))/4
    }
    if (m_HelperStick.getRawButton(9)) {
      m_shooter.moveToAmp();
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    m_DriveStick.setRumble(RumbleType.kBothRumble, 0);
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
    // m_elevator.teleopInit(); 
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // // SmartDashboard.putNumberArray("Limelight Distance Vector Magnitude", (Double[]) m_visionSystem.getDistancesMagnitudes().values().toArray());
    if (m_DriveStick.getRawButton(1)) {
      m_shooter.runShooter(-1);
      // SmartDashboard.putBoolean("run shooter", true);
    } else {
      m_shooter.runShooter(0);
      // SmartDashboard.putBoolean("run shooter", false);
    }
    // if (m_DriveStick.getRawButton(2)) {
    //   m_shooter.runIntake(-1);
    // } else {
    //   m_shooter.runIntake(0);
    // }
    // if (m_DriveStick.getRawButton(3)) {
    //   m_intake.runIntake(0.3);
    // } else if(m_DriveStick.getRawButton(4)){
    //   m_intake.runIntake(-0.3);
    // } else {
    //   m_intake.runIntake(0);
    // }
    m_shooter.runRotatingMotor(m_DriveStick.getRawAxis(1)/4);
    SmartDashboard.putNumber("shooter pivot power", m_DriveStick.getRawAxis(1)/4);
    // m_intake.runPitchMotor(m_DriveStick.getRawAxis(5)/4);
    // SmartDashboard.putNumber("intake pivot power", m_DriveStick.getRawAxis(5)/4);
    // m_intake.setAngle(ShooterIntakeConstants.kIntakeHigherPosition);
    if (m_DriveStick.getRawButton(3)) {
      m_intake.setPosition(true);
    } else {
      m_intake.setPosition(false);
    }
    
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
