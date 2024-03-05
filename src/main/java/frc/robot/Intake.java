package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;


public class Intake {
    private static CANSparkMax m_intakeMotor;
    private static CANSparkMax m_pitchMotor;
    private static CANcoder m_pitchEncoder;

    private static PIDController m_pitchController;

    private static DigitalInput m_limitSwitch;

    /**
     * Constructor that uses constants in constants file
     */
    public Intake() {
        m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorCanId, MotorType.kBrushless);
        m_pitchMotor = new CANSparkMax(IntakeConstants.kPitchMotorCanId, MotorType.kBrushless);
        m_pitchEncoder = new CANcoder(IntakeConstants.kPitchEncoderCanId);
        m_pitchController = new PIDController(IntakeConstants.kPitchP, IntakeConstants.kPitchI, IntakeConstants.kPitchD);
        m_limitSwitch = new DigitalInput(IntakeConstants.kLimitSwitchPort);
        m_pitchController.setTolerance(IntakeConstants.kIntakeTolerance);
    }

    public Rotation2d getAngle() {
        return new Rotation2d(m_pitchEncoder.getAbsolutePosition().getValueAsDouble() * 360);
    }

    /**
     * Gets the current value of the limit switch
     * @return whether the limit switch is pressed or not
     */
    public boolean getLimitSwitch() {
        return m_limitSwitch.get();
    }

    /**
     * Sets the intake to a certain angle
     * @param angle a Rotation2d containing the angle to set the intake to
     */
    public void setAngle(Rotation2d angle) {
        double cosineScalar = Math.cos(getAngle().getRadians());
        m_pitchController.setSetpoint(angle.getRadians());
        m_pitchMotor.set(m_pitchController.calculate(getAngle().getRadians()) + IntakeConstants.kPitchKG * cosineScalar);
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
