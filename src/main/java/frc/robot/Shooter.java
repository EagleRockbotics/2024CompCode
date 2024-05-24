package frc.robot;

import com.revrobotics.CANSparkFlex;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;


public class Shooter {
    private final WPI_VictorSPX m_shootingMotorLeft;
    private final WPI_VictorSPX m_shootingMotorRight;
    private final CANSparkFlex m_rotatingMotor;
    private final WPI_VictorSPX m_intakeMotor;
    private final CANcoder m_rotatingEncoder;
    private final PIDController m_rotatingController;

    /**
     * Constructor that uses constants defined in Constants.ShooterConstants
     */
    public Shooter() {
        m_shootingMotorLeft = new WPI_VictorSPX(ShooterConstants.kShootingLeftCanId);
        m_shootingMotorRight = new WPI_VictorSPX(ShooterConstants.kShootingRightCanId);
        m_rotatingMotor = new CANSparkFlex(ShooterConstants.kShootingRotatingCanId, MotorType.kBrushless);
        m_intakeMotor = new WPI_VictorSPX(ShooterConstants.kShooterIntakeCanId);
        m_rotatingEncoder = new CANcoder(ShooterConstants.kShooterRotatingEncoderCanId);
        m_rotatingController = new PIDController(ShooterConstants.kShootingRotatingP, ShooterConstants.kShootingRotatingI, ShooterConstants.kShootingRotatingD);
        m_rotatingController.setTolerance(ShooterConstants.kShooterTolerance);
        m_rotatingEncoder.setPosition(0);
    }

    public void resetAngle() {
        m_rotatingEncoder.setPosition(0);
    }
    // -.23
    public Rotation2d getAngle() {
        SmartDashboard.putNumber("Shooter Position", m_rotatingEncoder.getAbsolutePosition().getValueAsDouble());
        return Rotation2d.fromRotations(m_rotatingEncoder.getPosition().getValueAsDouble());
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
     * @param speed should be from -1 to 1, negative to feed to shooter, positive to feed backwards
     */
    public void runIntake(double speed) {
        m_intakeMotor.set(speed);
    }

    /**
     * Runs the intake motor to a specific speed for debugging purposes -.23
     * @param speed
     */
    public void runRotatingMotor(double speed) {
        m_rotatingMotor.set(-speed);
        if (-speed < 0 && getAngle().getRotations() > 0) {
            m_rotatingMotor.set(0);
        }
    }

    public void moveToAmp() {
        if (getAngle().getRotations() > -.22) {
            m_rotatingMotor.set(.25);
        } else {
            m_rotatingMotor.set(0);
        }
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
