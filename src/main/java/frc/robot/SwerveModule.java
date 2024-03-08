package frc.robot;

import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private final CANSparkMax m_drivingSparkMax;
    private final CANSparkMax m_turningSparkMax;

    private final RelativeEncoder m_drivingEncoder;
    private final CANcoder m_turningEncoder;

    private final SparkPIDController m_drivingPIDController;
    private final PIDController m_turningPIDController;

    private double driveDirection = 1;


    public SwerveModule(int drivingCANId, int turningCANId, int encoderNum, boolean reversedDrive,
            boolean reversedSteer) {
        m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
        m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

        // m_drivingSparkMax.setInverted(reversedDrive);
        // m_turningSparkMax.setInverted(reversedSteer);
        if (reversedDrive) {
            driveDirection = -1;
        }

        m_drivingSparkMax.restoreFactoryDefaults();
        m_turningSparkMax.restoreFactoryDefaults();

        m_drivingEncoder = m_drivingSparkMax.getEncoder();
        m_turningEncoder = new CANcoder(encoderNum);
        
        m_turningPIDController = new PIDController(ModuleConstants.kTurningP, ModuleConstants.kTurningI,
                ModuleConstants.kTurningD);
        m_drivingPIDController = m_drivingSparkMax.getPIDController();
        m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);

        m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
        m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

        m_turningPIDController.enableContinuousInput(
            ModuleConstants.kTurningEncoderPositionPIDMinInput,
            ModuleConstants.kTurningEncoderPositionPIDMaxInput
        );
        
        m_drivingPIDController.setP(ModuleConstants.kDrivingP);
        m_drivingPIDController.setI(ModuleConstants.kDrivingI);
        m_drivingPIDController.setD(ModuleConstants.kDrivingD);
        m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
        m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);

        m_drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
        m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
        m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

        m_drivingSparkMax.burnFlash();
        m_turningSparkMax.burnFlash();

        m_drivingEncoder.setPosition(0);
    }

    public Rotation2d getTurningAngle() {
        return Rotation2d.fromRadians(m_turningEncoder.getAbsolutePosition().getValueAsDouble() * Math.PI * 2);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveDirection * m_drivingEncoder.getVelocity(), getTurningAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveDirection * m_drivingEncoder.getPosition(), getTurningAngle());
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState, getTurningAngle());

        SmartDashboard.putNumber("Speed", optimizedDesiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("ActualSpeed", m_drivingEncoder.getVelocity());

        m_drivingPIDController.setReference(driveDirection*optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        m_turningSparkMax.set(
            -m_turningPIDController.calculate(
                getTurningAngle().getDegrees(), 
                optimizedDesiredState.angle.getDegrees()
            )
        );
    }

    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
        m_turningEncoder.setPosition(0);
    }

    public void runModulePowerControl(SwerveModuleState state) {
        SwerveModuleState optimised = SwerveModuleState.optimize(state, getTurningAngle());

        m_drivingSparkMax.set(driveDirection * optimised.speedMetersPerSecond);
        m_turningSparkMax.set(-m_turningPIDController.calculate(getTurningAngle().getDegrees(), optimised.angle.getDegrees()));
    }

    public void stopModule() {
        m_turningSparkMax.set(0);
        m_drivingSparkMax.set(0);
    }
}
