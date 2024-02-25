package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants.ElevatorConstants;

public class Elevator {
    private final WPI_TalonSRX m_LeftElevator;
    private final WPI_TalonSRX m_RightElevator;
    private final CANcoder m_LeftCanCoder;
    private final CANcoder m_RightCanCoder;
    private String m_CurrentCommand = ElevatorConstants.kElevatorIdle;
    private double m_elevatorSpeedLeft = 0;
    private double m_elevatorSpeedRight = 0;

    /**
     * Constructor using constants in constants file
     */
    public Elevator() {
        m_LeftElevator = new WPI_TalonSRX(ElevatorConstants.kLeftElevatorCanID);
        m_RightElevator = new WPI_TalonSRX(ElevatorConstants.kRightElevatorCanID);
        m_LeftCanCoder = new CANcoder(ElevatorConstants.kLeftElevatorEncoderCanId);
        m_RightCanCoder = new CANcoder(ElevatorConstants.kRightElevatorEncoderCanId);
    }

    private void setMaxHeight() {
        if (m_LeftCanCoder.getPosition().getValueAsDouble() < ElevatorConstants.kElevatorMaxHeight) {
            m_LeftElevator.set(1);
        } else {
            m_LeftElevator.set(0);
        }
        if (m_RightCanCoder.getPosition().getValueAsDouble() < ElevatorConstants.kElevatorDownHeight) {
            m_RightElevator.set(1);
        } else {
            m_RightElevator.set(0);
        }
    }

    private void setMinHeight() {
        if (m_LeftCanCoder.getPosition().getValueAsDouble() < 0) {
            m_LeftElevator.set(1);
        } else {
            m_LeftElevator.set(0);
        }
        if (m_RightCanCoder.getPosition().getValueAsDouble() < 0) {
            m_RightElevator.set(1);
        } else {
            m_RightElevator.set(0);
        }
    }

    private void manual(double leftSpeed, double rightSpeed) {
        if (m_LeftCanCoder.getPosition().getValueAsDouble() < 0) {
            m_LeftElevator.set(leftSpeed);
        } else {
            m_LeftElevator.set(0);
        }
        if (m_RightCanCoder.getPosition().getValueAsDouble() < 0) {
            m_RightElevator.set(rightSpeed);
        } else {
            m_RightElevator.set(0);
        }
    }

    private void idle() {
        m_LeftElevator.set(0);
        m_RightElevator.set(0);
    }

    /**
     * Sets the speed of the elevator for manual control
     * @param speed the speed of the elevator going down, from 0 to 1
     */
    public void setElevatorSpeed(double leftSpeed, double rightSpeed) {
        m_elevatorSpeedLeft = leftSpeed;
        m_elevatorSpeedRight = rightSpeed;
    }

    /**
     * Function to call every loop
     */
    public void periodic() {
        if (m_CurrentCommand == ElevatorConstants.kRelease) {
            setMinHeight();
        } else if (m_CurrentCommand == ElevatorConstants.kReturn) {
            setMaxHeight();
        } else if (m_CurrentCommand == ElevatorConstants.kElevatorManual) {
            manual(m_elevatorSpeedLeft, m_elevatorSpeedRight);
        } else {
            idle();
        }
    }

    /**
     * Function to change the current command
     * @param newCommand Takes in ElevatorConstants.(kElevatorIdle, kElevatorMin, kElevatorMax)
     */
    public void changeCommand(String newCommand) {
        m_CurrentCommand = newCommand;
    }
}
