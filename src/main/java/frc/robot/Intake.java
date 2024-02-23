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

    
}
