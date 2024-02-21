package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;


public class Shooter {
    private final CANSparkMax m_shootingMotorLeft;
    private final CANSparkMax m_shootingMotorRight;
    private final CANSparkMax m_rotatingMotor;
    private final CANSparkMax m_intakeMotor;
    private final CANcoder m_rotatingEncoder;
    private final PIDController m_rotatingController;
    private final Timer m_timer = new Timer();

    public Shooter(int shootingMotorLeftCanId, int shootingMotorRightCanId, int rotatingMotorCanId, int rotatingEncoderCanId, int intakeMotorCanId) {
        m_shootingMotorLeft = new CANSparkMax(shootingMotorLeftCanId, MotorType.kBrushless);
        m_shootingMotorRight = new CANSparkMax(shootingMotorRightCanId, MotorType.kBrushless);
        m_rotatingMotor = new CANSparkMax(rotatingMotorCanId, MotorType.kBrushless);
        m_intakeMotor = new CANSparkMax(intakeMotorCanId, MotorType.kBrushless);
        m_rotatingEncoder = new CANcoder(rotatingEncoderCanId);
        m_rotatingController = new PIDController(ShooterConstants.kShootingRotatingP, ShooterConstants.kShootingRotatingI, ShooterConstants.kShootingRotatingD);
        m_rotatingController.setTolerance(ShooterConstants.kShooterTolerance);
    }

    public Shooter() {
        m_shootingMotorLeft = new CANSparkMax(ShooterConstants.kShootingLeftCanId, MotorType.kBrushless);
        m_shootingMotorRight = new CANSparkMax(ShooterConstants.kShootingRightCanId, MotorType.kBrushless);
        m_rotatingMotor = new CANSparkMax(ShooterConstants.kShootingRotatingCanId, MotorType.kBrushless);
        m_intakeMotor = new CANSparkMax(ShooterConstants.kShooterIntakeCanId, MotorType.kBrushless);
        m_rotatingEncoder = new CANcoder(ShooterConstants.kShooterRotatingEncoderCanId);
        m_rotatingController = new PIDController(ShooterConstants.kShootingRotatingP, ShooterConstants.kShootingRotatingI, ShooterConstants.kShootingRotatingD);
        m_rotatingController.setTolerance(ShooterConstants.kShooterTolerance);
    }

    public double getAngleRadians() {
        return m_rotatingEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
    }

    public double getAngleDegrees() {
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
     * @param tryShooting If false is passed into this value, the shooter will be lowered to its default position, and motor will stop running
     * @param distanceFromTarget The distance from the target
     */
    public void runShooter(double distanceFromTarget, boolean tryShooting) {
        if (tryShooting) {
            m_timer.start();
            double angle = calculateAngle(distanceFromTarget);
            setAngle(angle);
            m_shootingMotorLeft.set(1);
            m_shootingMotorRight.set(-1);
            if (m_timer.get() > 4) {
                // run intake into shooter. This function may be moved into a class containing both the shooter and intake systems
            }
        } else {
            m_timer.stop();
            m_timer.reset();
            m_shootingMotorLeft.set(0);
            m_shootingMotorRight.set(0);
            setAngle(ShooterConstants.kDefaultShooterPosition);
        }
    }
}
