package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.networktables.*;
import edu.wpi.first.math.controller.PIDController;

import org.opencv.core.Mat.Tuple3;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ControllerConstants;

public class SwerveDrive implements Subsystem {
    private final SwerveModule m_FLSwerve;
    private final SwerveModule m_FRSwerve;
    private final SwerveModule m_RLSwerve;
    private final SwerveModule m_RRSwerve;

    private final PIDController autoTargetingController;

    private SwerveDriveOdometry m_odometry;

    public final Pigeon2 m_gyro;

    private StructArrayPublisher<SwerveModuleState> publisherRed;
    private StructArrayPublisher<SwerveModuleState> publisherBlue;

    private Field2d m_field2d = new Field2d();

    public SwerveDrive() {
        m_FLSwerve = new SwerveModule(DriveConstants.kFrontLeftDrivingCanId, DriveConstants.kFrontLeftTurningCanId,
                DriveConstants.kFrontLeftEncoderCanId, ModuleConstants.FL_driveInverted,
                ModuleConstants.FL_steerInverted);
        m_FRSwerve = new SwerveModule(DriveConstants.kFrontRightDrivingCanId, DriveConstants.kFrontRightTurningCanId,
                DriveConstants.kFrontRightEncoderCanId, ModuleConstants.FR_driveInverted,
                ModuleConstants.FR_steerInverted);
        m_RLSwerve = new SwerveModule(DriveConstants.kRearLeftDrivingCanId, DriveConstants.kRearLeftTurningCanId,
                DriveConstants.kRearLeftEncoderCanId, ModuleConstants.RL_driveInverted,
                ModuleConstants.RL_steerInverted);
        m_RRSwerve = new SwerveModule(DriveConstants.kRearRightDrivingCanId, DriveConstants.kRearRightTurningCanId,
                DriveConstants.kRearRightEncoderCanId, ModuleConstants.RR_driveInverted,
                ModuleConstants.RR_steerInverted);

        m_gyro = new Pigeon2(DriveConstants.kPigeonGyro);
        publishStates();

        m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
                getHeading(),
                new SwerveModulePosition[] {
                        m_FLSwerve.getPosition(),
                        m_FRSwerve.getPosition(),
                        m_RLSwerve.getPosition(),
                        m_RRSwerve.getPosition()
                }, new Pose2d(new Translation2d(0, 0), Rotation2d.fromRadians(0)));

        // AutoBuilder.configureHolonomic(this::getPose, this::resetPose, this::getCurrentRobotRelativeSpeeds,
        //         this::driveRobotRelative, new HolonomicPathFollowerConfig(new PIDConstants(DriveConstants.TkP, DriveConstants.TkI, DriveConstants.TkD),
        //                 new PIDConstants(DriveConstants.RkP, DriveConstants.RkI, DriveConstants.RkD), 4.5,
        //                 Math.sqrt(Math.pow(DriveConstants.kTrackWidth, 2) + Math.pow(DriveConstants.kWheelBase, 2)),
        //                 new ReplanningConfig()),
        //         () -> {
        //             var alliance = DriverStation.getAlliance();
        //             if (alliance.isPresent()) {
        //                 return alliance.get() == DriverStation.Alliance.Red;
        //             }
        //             return false;
        //         }, this);
        SmartDashboard.putData("Field View", m_field2d);

        autoTargetingController = new PIDController(DriveConstants.kAutoTargettingP, DriveConstants.kAutoTargettingI,
                DriveConstants.kAutoTargettingD);
    }

    public void publishStates() {
        publisherRed = NetworkTableInstance.getDefault()
                .getStructArrayTopic("/SwerveStates/Red", SwerveModuleState.struct)
                .publish();
        publisherBlue = NetworkTableInstance.getDefault()
                .getStructArrayTopic("/SwerveStates/Blue", SwerveModuleState.struct).publish();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(m_gyro.getAngle() + 90);
    }

    public double getTurnRate() {
        return (m_gyro.getRate()) * (((DriveConstants.kGyroReversed ? 1.0 : -1.0) - 0.5) / .5);
    }

    public void zeroHeading() {
        m_gyro.reset();
    }

    public void resetEncoders() {
        m_FLSwerve.resetEncoders();
        m_FRSwerve.resetEncoders();
        m_RLSwerve.resetEncoders();
        m_RRSwerve.resetEncoders();
    }

    public void setModuleStates(edu.wpi.first.math.kinematics.SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

        m_FLSwerve.runModulePowerControl(desiredStates[0]);
        m_FRSwerve.runModulePowerControl(desiredStates[1]);
        m_RLSwerve.runModulePowerControl(desiredStates[2]);
        m_RRSwerve.runModulePowerControl(desiredStates[3]);

        publisherRed.set(new SwerveModuleState[] {
                desiredStates[0], desiredStates[1], desiredStates[2], desiredStates[3]
        });
        publisherBlue.set(new SwerveModuleState[] { m_FLSwerve.getState(), m_FRSwerve.getState(), m_RLSwerve.getState(),
                m_RRSwerve.getState() }); 

        m_odometry.update(
            getHeading(), 
            new SwerveModulePosition[] { 
                m_FLSwerve.getPosition(),
                m_FRSwerve.getPosition(), 
                m_RLSwerve.getPosition(), 
                m_RRSwerve.getPosition() 
            }
        );
        m_field2d.setRobotPose(m_odometry.getPoseMeters());

        SmartDashboard.putNumber("x Position", m_odometry.getPoseMeters().getX());
    }

    /**
     * Locks wheels in place
     */
    public void setX() {
        m_FLSwerve.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_FRSwerve.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_RLSwerve.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_RRSwerve.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Command for driving relative to field
     */
    public void driveFieldRelative(double xSpeed, double ySpeed, double rot) {
        SmartDashboard.putNumber("Gyro Position", getHeading().getDegrees());
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, rot, getHeading());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

    /**
     * Command for driving relative to robot
     * 
     * @param speeds
     */
    public void driveRobotRelative(double xSpeed, double ySpeed, double rot) {
        SmartDashboard.putNumber("gyro", getHeading().getDegrees());
        ChassisSpeeds speeds = new ChassisSpeeds(ySpeed, -xSpeed, rot);
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(moduleStates);
    }

    /**
     * Stops wheels from moving
     */
    public void stopModules() {
        m_FRSwerve.stopModule();
        m_FLSwerve.stopModule();
        m_RLSwerve.stopModule();
        m_RRSwerve.stopModule();
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            m_FLSwerve.getPosition(),
            m_FRSwerve.getPosition(),
            m_RLSwerve.getPosition(),
            m_RRSwerve.getPosition()
        };
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            m_FLSwerve.getState(),
            m_FRSwerve.getState(),
            m_RLSwerve.getState(),
            m_RRSwerve.getState(),
        };
    }

    public void resetPose(Pose2d pose) {
        m_odometry.resetPosition(getHeading(), getModulePositions(), pose);
    }

    public ChassisSpeeds getCurrentRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Converts axis on controller to speeds
     * 
     * @param xAxis the x axis of the controller
     * @param yAxis the y axis of the controller
     * @param zAxis the z axis of the controller
     * @return outputs an array containg [x speed, y speed, rotation speed]
     */
    public double[] JoystickConverter(double xAxis, double yAxis, double zAxis) {
        double[] XYRotValues = new double[3];

        XYRotValues[0] = xAxis;
        XYRotValues[1] = yAxis;
        XYRotValues[2] = zAxis;

        // apply deadzones
        if (Math.abs(zAxis) <= ControllerConstants.kSteerDeadzone) {
            XYRotValues[2] = 0;
        }
        if (Math.abs(yAxis) <= ControllerConstants.kDriveDeadzone) {
            XYRotValues[1] = 0;
        }
        if (Math.abs(xAxis) <= ControllerConstants.kDriveDeadzone) {
            XYRotValues[0] = 0;
        }

        XYRotValues[0] *= -ControllerConstants.kDrivingSpeed;
        XYRotValues[1] *= ControllerConstants.kDrivingSpeed;
        XYRotValues[2] *= -Math.PI * ControllerConstants.kSteerSpeed;

        return XYRotValues;
    }

    /**
     * Function for autoAiming towards a tag
     * 
     * @param angleToTag the angle between the robot's current rotation and the
     *                   degree position
     * @return returns a new rot value
     */
    public double AutoTargeter(double angleToTag) {
        if (angleToTag > 899) {
            return Math.PI / 2;
        }
        return autoTargetingController.calculate(angleToTag);
    }
}