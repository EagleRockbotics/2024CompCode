package frc.robot;

import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;


public class Intake {
    private static CANSparkMax m_intakeMotor;
    private static CANSparkMax m_pitchMotor;
    private static CANcoder m_pitchEncoder;

    /**
     * Constructor that uses constants in constants file
     */
    public Intake() {
        m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorCanId, MotorType.kBrushless);
        m_pitchMotor = new CANSparkMax(IntakeConstants.kPitchMotorCanId, MotorType.kBrushless);
        m_pitchEncoder = new CANcoder(IntakeConstants.kPitchEncoderCanId);
    }

    /**
     * Constructor that uses passed in Can Ids
     * @param IntakeMotorCanId
     * @param PitchMotorCanId
     * @param PitchEncoderCanId
     */
    public Intake(int IntakeMotorCanId, int PitchMotorCanId, int PitchEncoderCanId) {
        m_intakeMotor = new CANSparkMax(IntakeMotorCanId, MotorType.kBrushless);
        m_pitchMotor = new CANSparkMax(PitchMotorCanId, MotorType.kBrushless);
        m_pitchEncoder = new CANcoder(PitchEncoderCanId);
    }

    private void lowerIntake() {
        if (m_pitchEncoder.getAbsolutePosition().getValueAsDouble() > IntakeConstants.kPitchLow) {
            m_pitchMotor.set(-1);
        }
    }

    private void raiseIntake() {
        if (m_pitchEncoder.getAbsolutePosition().getValueAsDouble() < IntakeConstants.kPitchHigh) {
            m_pitchMotor.set(1);
        }
    }

    private void runIntake() {
        m_intakeMotor.set(1);
    }

    private void stopIntake() {
        m_intakeMotor.set(0);
    }
}
