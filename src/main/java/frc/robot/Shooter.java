package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;


public class Shooter {
    private final CANSparkMax m_shootingMotor;
    private final CANSparkMax m_rotatingMotor;
    private final CANcoder m_rotatingEncoder;
    private final PIDController m_rotatingController;

    public Shooter(int shootingMotorCanId, int rotatingMotorCanId, int rotatingEncoderCanId) {
        m_shootingMotor = new CANSparkMax(shootingMotorCanId, MotorType.kBrushless);
        m_rotatingMotor = new CANSparkMax(rotatingMotorCanId, MotorType.kBrushless);
        m_rotatingEncoder = new CANcoder(rotatingEncoderCanId);
        m_rotatingController = new PIDController(ShooterConstants.kShootingRotatingP, ShooterConstants.kShootingRotatingI, ShooterConstants.kShootingRotatingD);
        m_rotatingController.setTolerance(ShooterConstants.kShooterTolerance);
    }

    public Shooter() {
        m_shootingMotor = new CANSparkMax(ShooterConstants.kShootingCanId, MotorType.kBrushless);
        m_rotatingMotor = new CANSparkMax(ShooterConstants.kShootingRotatingCanId, MotorType.kBrushless);
        m_rotatingEncoder = new CANcoder(ShooterConstants.kShooterRotatingEncoderCanId);
        m_rotatingController = new PIDController(ShooterConstants.kShootingRotatingP, ShooterConstants.kShootingRotatingI, ShooterConstants.kShootingRotatingD);
        m_rotatingController.setTolerance(ShooterConstants.kShooterTolerance);
    }

    private double getAngleRadians() {
        return m_rotatingEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
    }

    private double getAngleDegrees() {
        return m_rotatingEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    }

    private void setAngle(double targetAngle) {
        m_rotatingMotor.set(m_rotatingController.calculate(targetAngle, getAngleDegrees()));
    }

    private double calculateAngle(double distanceFromTarget) {
        // can -90 degrees to 90 degrees, but should only return ~15-45 degrees
        return Math.atan((FieldConstants.kSpeakerHeight-RobotConstants.kShooterHeight)/distanceFromTarget) * 180/Math.PI;
    }

    /**
     * Runs the shooter.
     * This assumes that the shooter is pointing directly at the target already, 
     * other PID controllers may have to be used to make sure this happens
     * @param tryShooting If false is passed into this value, the shooter will be lowered to its default position
     * @param distanceFromTarget The distance from the target
     */
    public void runShooter(double distanceFromTarget, boolean tryShooting) {
        if (tryShooting) {
            double angle = calculateAngle(distanceFromTarget);
            setAngle(angle);
            if (m_rotatingController.atSetpoint()) {
                m_shootingMotor.set(1);
            } else {
                m_shootingMotor.set(0);
            }
        } else {
            m_shootingMotor.set(0);
            setAngle(ShooterConstants.kDefaultShooterPosition);
        }
    }
}
