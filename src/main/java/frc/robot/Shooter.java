package frc.robot;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.Constants.ShooterConstants;


public class Shooter {
    private final CANSparkMax m_shootingMotorLeft;
    private final CANSparkMax m_shootingMotorRight;
    private final CANSparkFlex m_rotatingMotor;
    private final CANSparkMax m_intakeMotor;
    private final CANcoder m_rotatingEncoder;
    private final PIDController m_rotatingController;

    /**
     * Constructor that uses constants defined in Constants.ShooterConstants
     */
    public Shooter() {
        m_shootingMotorLeft = new CANSparkMax(ShooterConstants.kShootingLeftCanId, MotorType.kBrushless);
        m_shootingMotorRight = new CANSparkMax(ShooterConstants.kShootingRightCanId, MotorType.kBrushless);
        m_rotatingMotor = new CANSparkFlex(ShooterConstants.kShootingRotatingCanId, MotorType.kBrushless);
        m_intakeMotor = new CANSparkMax(ShooterConstants.kShooterIntakeCanId, MotorType.kBrushless);
        m_rotatingEncoder = new CANcoder(ShooterConstants.kShooterRotatingEncoderCanId);
        m_rotatingController = new PIDController(ShooterConstants.kShootingRotatingP, ShooterConstants.kShootingRotatingI, ShooterConstants.kShootingRotatingD);
        m_rotatingController.setTolerance(ShooterConstants.kShooterTolerance);
    }

    public Rotation2d getAngle() {
        return new Rotation2d(m_rotatingEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI * ShooterConstants.kShooterEncoderRatio);
    }

    /**
     * Runs the shooter motors
     * @param speed should be from -1 to 1, positive to shoot forward, negative to go backwards
     */
    public void runShooter(double speed) {
        m_shootingMotorLeft.set(speed);
        m_shootingMotorRight.set(-speed);
    }

    /**
     * Runs the intake motors
     * @param speed should be from -1 to 1, positive to feed to shooter, negative to feed backwards
     */
    public void runIntake(double speed) {
        m_intakeMotor.set(speed);
    }

    /**
     * Runs the intake motor to a specific speed for debugging purposes
     * @param speed
     */
    public void runRotatingMotor(double speed) {
        m_rotatingMotor.set(speed);
    }

    public Boolean atSetPoint() {
        return m_rotatingController.atSetpoint();
    }

    /**
     * Sets the position of the arm
     * @param targetAngle a Rotation2d containing the angle to set the arm to, 0 to go straight outwards
     */
    public void setAngle(Rotation2d targetAngle) {
        double cosineScalar = Math.cos(getAngle().getRadians());
        m_rotatingController.setSetpoint(targetAngle.getRadians());
        m_rotatingMotor.set(m_rotatingController.calculate(getAngle().getRadians()) + ShooterConstants.kShootingKG * cosineScalar);
    }
}
