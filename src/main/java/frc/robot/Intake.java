package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterIntakeConstants;


public class Intake {
    private static CANSparkMax m_intakeMotor;
    private static CANSparkMax m_pitchMotor;
    private static CANcoder m_pitchEncoder;

    private static PIDController m_pitchController;

    /**
     * Constructor that uses constants in constants file
     */
    public Intake() {
        m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorCanId, MotorType.kBrushless);
        m_pitchMotor = new CANSparkMax(IntakeConstants.kPitchMotorCanId, MotorType.kBrushed);
        m_pitchEncoder = new CANcoder(IntakeConstants.kPitchEncoderCanId);
        m_pitchController = new PIDController(IntakeConstants.kPitchP, IntakeConstants.kPitchI, IntakeConstants.kPitchD);
        m_pitchController.setTolerance(IntakeConstants.kIntakeTolerance);
        m_pitchEncoder.setPosition(0);
    }

    public void resetEncoder() {
        m_pitchEncoder.setPosition(0);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(-m_pitchEncoder.getAbsolutePosition().getValueAsDouble());
    }

    /**
     * Sets the intake to a certain angle
     * @param angle a Rotation2d containing the angle to set the intake to
     */
    public void setAngle(Rotation2d angle) {
        if (angle.getDegrees() < ShooterIntakeConstants.kIntakeLowerPosition.getDegrees()) {
            angle = ShooterIntakeConstants.kIntakeLowerPosition;
        } else if (angle.getDegrees() > ShooterIntakeConstants.kIntakeHigherPosition.getDegrees()) {
            angle = ShooterIntakeConstants.kIntakeHigherPosition;
        }
        if (angle.getDegrees() == getAngle().getDegrees()) {
            SmartDashboard.putBoolean("equal", true);
        } else {
            SmartDashboard.putBoolean("equal", false);
        }
        double cosineScalar = Math.cos(getAngle().getRadians());
        m_pitchController.setSetpoint(angle.getRadians());
        SmartDashboard.putNumber("PID Intake Power", IntakeConstants.kPitchKG * cosineScalar);
        m_pitchMotor.set(m_pitchController.calculate(getAngle().getRadians()) + IntakeConstants.kPitchKG * cosineScalar);
    }

    public void setPosition(boolean intakeDown) {
        if (intakeDown && getAngle().getRotations() < ShooterIntakeConstants.kIntakeLowerPosition.getRotations()) {
            m_pitchMotor.set(.4);
        } else if (!intakeDown && getAngle().getRotations() > ShooterIntakeConstants.kIntakeHigherPosition.getRotations()) {
            m_pitchMotor.set(-.5);
        } else {
            m_pitchMotor.set(0);
        }
        SmartDashboard.putBoolean("up position intake", intakeDown);
        SmartDashboard.putNumber("intake Position testing", getAngle().getRotations());
    }

    public Boolean atSetPoint() {
        return m_pitchController.atSetpoint();
    }

    /**
     * runs the intake at a certain speed
     * @param speed should be -1 to 1, positive to intake, negative to outtake
     */
    public void runIntake(double speed) {
        m_intakeMotor.set(speed);
    }

    /**
     * runs the pitch motor for testing purposes
     * @param speed the speed of the motor, should be -1 to 1
     */
    public void runPitchMotor(double speed) {
        m_pitchMotor.set(speed);
    }
}
