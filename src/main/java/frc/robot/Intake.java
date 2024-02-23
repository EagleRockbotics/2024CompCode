package frc.robot;


import edu.wpi.first.math.controller.PIDController;
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

    private String m_currentCommand = IntakeConstants.kIdle;

    /**
     * Constructor that uses constants in constants file
     */
    public Intake() {
        m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorCanId, MotorType.kBrushless);
        m_pitchMotor = new CANSparkMax(IntakeConstants.kPitchMotorCanId, MotorType.kBrushless);
        m_pitchEncoder = new CANcoder(IntakeConstants.kPitchEncoderCanId);
        m_pitchController = new PIDController(IntakeConstants.kPitchP, IntakeConstants.kPitchI, IntakeConstants.kPitchD);
        m_limitSwitch = new DigitalInput(IntakeConstants.kLimitSwitchPort);
    }

    /**
     * Constructor that uses passed in Can Ids
     * @param IntakeMotorCanId
     * @param PitchMotorCanId
     * @param PitchEncoderCanId
     */
    public Intake(int IntakeMotorCanId, int PitchMotorCanId, int PitchEncoderCanId, int LimitSwitchPort) {
        m_intakeMotor = new CANSparkMax(IntakeMotorCanId, MotorType.kBrushless);
        m_pitchMotor = new CANSparkMax(PitchMotorCanId, MotorType.kBrushless);
        m_pitchEncoder = new CANcoder(PitchEncoderCanId);
        m_pitchController = new PIDController(IntakeConstants.kPitchP, IntakeConstants.kPitchI, IntakeConstants.kPitchD);
        m_limitSwitch = new DigitalInput(LimitSwitchPort);
    }

    /**
     * Function for changing the current command. If setting the current command to kLoadShooter, make sure that you also set the shooter to the load shooter command
     * @param command Takes IntakeConstants.(kIdle, kLoadShooter)
     */
    public void setCommand(String command) {
        m_currentCommand = command;
    }

    /**
     * Function to be called every loop
     */
    public void periodic() {
        if (m_currentCommand == IntakeConstants.kLoadShooter) {
            if (!m_limitSwitch.get()) {
                m_pitchController.setSetpoint(IntakeConstants.kPitchLow);
                if (!m_pitchController.atSetpoint()) {
                    m_pitchMotor.set(m_pitchController.calculate(m_pitchEncoder.getAbsolutePosition().getValueAsDouble()));
                    m_intakeMotor.set(1);
                } else {
                    m_pitchMotor.set(0);
                    m_intakeMotor.set(1);
                }
            } else {
                m_pitchController.setSetpoint(IntakeConstants.kPitchHigh);
                if (!m_pitchController.atSetpoint()) {
                    m_pitchMotor.set(m_pitchController.calculate(m_pitchEncoder.getAbsolutePosition().getValueAsDouble()));
                    m_intakeMotor.set(0);
                } else {
                    m_pitchMotor.set(0);
                    m_intakeMotor.set(0);
                    m_currentCommand = IntakeConstants.kIdle;
                }
            }
        } else {
            m_pitchController.setSetpoint(IntakeConstants.kPitchHigh);
            if (!m_pitchController.atSetpoint()) {
                m_pitchMotor.set(m_pitchController.calculate(m_pitchEncoder.getAbsolutePosition().getValueAsDouble()));
                m_intakeMotor.set(0);
            } else {
                m_pitchMotor.set(0);
                m_intakeMotor.set(0);
            }
        }
    }
}
