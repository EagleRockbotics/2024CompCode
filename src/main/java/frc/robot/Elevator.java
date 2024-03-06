package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ElevatorConstants;

public class Elevator {
    private final CANSparkMax m_LeftElevator;
    private final CANSparkMax m_RightElevator;
    private final CANcoder m_LeftCanCoder;
    private final CANcoder m_RightCanCoder;
    // private String m_CurreCANntCommand = ElevatorConstants.kElevatorIdle;
    // private double m_elevatorSpeedLeft = 0;
    // private double m_elevatorSpeedRight = 0;

    /**
     * Constructor using constants in constants file
     */
    public Elevator() {
        m_LeftElevator = new CANSparkMax(ElevatorConstants.kLeftElevatorCanID, MotorType.kBrushed);
        m_RightElevator = new CANSparkMax(ElevatorConstants.kRightElevatorCanID, MotorType.kBrushed);
        m_LeftCanCoder = new CANcoder(ElevatorConstants.kLeftElevatorEncoderCanId);
        m_RightCanCoder = new CANcoder(ElevatorConstants.kRightElevatorEncoderCanId);
    }

    public double[] getPositions() {
        double[] positions = {m_LeftCanCoder.getPosition().getValueAsDouble(), m_RightCanCoder.getPosition().getValueAsDouble()};
        return positions;
    }

    public void setMaxHeight() {
        if (m_LeftCanCoder.getPosition().getValueAsDouble() < ElevatorConstants.kElevatorMaxHeight) {
            m_LeftElevator.set(1);
        } else {
            m_LeftElevator.set(0);
        }
        if (m_RightCanCoder.getPosition().getValueAsDouble() > ElevatorConstants.kElevatorMaxHeight) {
            m_RightElevator.set(-1);
        } else {
            m_RightElevator.set(0);
        }
    }

    public void setMinHeight() {
        if (m_LeftCanCoder.getPosition().getValueAsDouble() < ElevatorConstants.kElevatorDownHeight) {
            m_LeftElevator.set(1);
        } else {
            m_LeftElevator.set(0);
        }
        if (m_RightCanCoder.getPosition().getValueAsDouble() > -ElevatorConstants.kElevatorDownHeight) {
            m_RightElevator.set(-1);
        } else {
            m_RightElevator.set(0);
        }
    }

    public void manual(double leftSpeed, double rightSpeed) {
        if (m_LeftCanCoder.getPosition().getValueAsDouble() < ElevatorConstants.kElevatorDownHeight) {
            m_LeftElevator.set(-leftSpeed);
        } else {
            m_LeftElevator.set(0);
        }
        if (m_RightCanCoder.getPosition().getValueAsDouble() > -ElevatorConstants.kElevatorDownHeight) {
            m_RightElevator.set(rightSpeed);
        } else {
            m_RightElevator.set(0);
        }
    }

    public void HuenemeComp(double leftSpeed, double rightSpeed) {
        m_LeftElevator.set(-leftSpeed);
        m_RightElevator.set(rightSpeed);
    }

    public void idle() {
        m_LeftElevator.set(0);
        m_RightElevator.set(0);
    }

    // /**
    //  * Sets the speed of the elevator for manual control
    //  * @param speed the speed of the elevator going down, from 0 to 1
    //  */
    // public void setElevatorSpeed(double leftSpeed, double rightSpeed) {
    //     m_elevatorSpeedLeft = leftSpeed;
    //     m_elevatorSpeedRight = rightSpeed;
    // }

    // /**
    //  * Function to call every loop
    //  */
    // public void periodic() {
    //     if (m_CurrentCommand == ElevatorConstants.kRelease) {
    //         setMinHeight();
    //     } else if (m_CurrentCommand == ElevatorConstants.kReturn) {
    //         setMaxHeight();
    //     } else if (m_CurrentCommand == ElevatorConstants.kElevatorManual) {
    //         manual(m_elevatorSpeedLeft, m_elevatorSpeedRight);
    //     } else {
    //         idle();
    //     }
    // }

    // /**
    //  * Function to change the current command
    //  * @param newCommand Takes in ElevatorConstants.(kElevatorIdle, kElevatorMin, kElevatorMax)
    //  */
    // public void changeCommand(String newCommand) {
    //     m_CurrentCommand = newCommand;
    // }

    /**
     * Function to be called on teleop init
     */
    public void teleopInit() {
        m_LeftCanCoder.setPosition(-ElevatorConstants.kElevatorDownHeight);
        m_RightCanCoder.setPosition(ElevatorConstants.kElevatorDownHeight);
    }
}
