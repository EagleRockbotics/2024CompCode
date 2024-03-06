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

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveDrive implements Subsystem {
    private final SwerveModule m_FLSwerve;
    private final SwerveModule m_FRSwerve;
    private final SwerveModule m_RLSwerve;
    private final SwerveModule m_RRSwerve;

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
                m_gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        m_FLSwerve.getPosition(),
                        m_FRSwerve.getPosition(),
                        m_RLSwerve.getPosition(),
                        m_RRSwerve.getPosition()
                }, new Pose2d(new Translation2d(0, 0), Rotation2d.fromRadians(0)));

        AutoBuilder.configureHolonomic(this::getPose, this::resetPose, this::getCurrentRobotRelativeSpeeds,
                this::driveRobotRelative, new HolonomicPathFollowerConfig(new PIDConstants(DriveConstants.TkP, DriveConstants.TkI, DriveConstants.TkD),
                        new PIDConstants(DriveConstants.RkP, DriveConstants.RkI, DriveConstants.RkD), 4.5,
                        Math.sqrt(Math.pow(DriveConstants.kTrackWidth, 2) + Math.pow(DriveConstants.kWheelBase, 2)),
                        new ReplanningConfig()),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, this);
        SmartDashboard.putData("Field View", m_field2d);

    }

    public void publishStates() {
        publisherRed = NetworkTableInstance.getDefault()
                .getStructArrayTopic("/SwerveStates/Red", SwerveModuleState.struct)
                .publish();
        publisherBlue = NetworkTableInstance.getDefault()
                .getStructArrayTopic("/SwerveStates/Blue", SwerveModuleState.struct).publish();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(m_gyro.getAngle());
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

        m_FLSwerve.runModuleOptimised(desiredStates[0]);
        m_FRSwerve.runModuleOptimised(desiredStates[1]);
        m_RLSwerve.runModuleOptimised(desiredStates[2]);
        m_RRSwerve.runModuleOptimised(desiredStates[3]);

        publisherRed.set(new SwerveModuleState[] {
                desiredStates[0], desiredStates[1], desiredStates[2], desiredStates[3]
        });
        publisherBlue.set(new SwerveModuleState[] { m_FLSwerve.getState(), m_FRSwerve.getState(), m_RLSwerve.getState(),
                m_RRSwerve.getState() }); 

        m_odometry.update(Rotation2d.fromRadians(((m_gyro.getAngle()) * Math.PI)/180), new SwerveModulePosition[] { m_FLSwerve.getPosition(),
                m_FRSwerve.getPosition(), m_RLSwerve.getPosition(), m_RRSwerve.getPosition() });
        m_field2d.setRobotPose(m_odometry.getPoseMeters());

    }

    public void setX() {
        m_FLSwerve.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_FRSwerve.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_RLSwerve.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_RRSwerve.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public void driveFieldRelative(double xSpeed, double ySpeed, double rot) {
        SmartDashboard.putNumber("Gyro Position", m_gyro.getAngle() * Math.PI / 180);
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, rot,
                Rotation2d.fromRadians((m_gyro.getAngle() + 90) * Math.PI / 180));
        SmartDashboard.putNumber("gyro", m_gyro.getAngle());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SmartDashboard.putNumber("gyro", m_gyro.getAngle());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(moduleStates);
    }

    public void PIDTuningHelper(double angle, double speed) {
        m_FRSwerve.testModule(angle, speed);
        m_FLSwerve.testModule(angle, speed);
        m_RLSwerve.testModule(angle, speed);
        m_RRSwerve.testModule(angle, speed);
    }

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
        m_odometry.resetPosition(Rotation2d.fromRadians(((m_gyro.getAngle()) * Math.PI)/180), getModulePositions(), pose);
    }

    public ChassisSpeeds getCurrentRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

}