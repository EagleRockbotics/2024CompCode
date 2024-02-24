package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.FieldConstants;


public class Shooter {
    private final CANSparkMax m_shootingMotorLeft;
    private final CANSparkMax m_shootingMotorRight;
    private final CANSparkMax m_rotatingMotor;
    private final CANSparkMax m_intakeMotor;
    private final CANcoder m_rotatingEncoder;
    private final PIDController m_rotatingController;
    private final Timer m_timer = new Timer();
    private final Timer m_timer2 = new Timer();
    private final DigitalInput m_limitSwitch;
    // the current command being run
    private String m_currentCommand = ShooterConstants.kRest;
    // the current distance from target, should be continually updated if possible
    private double m_disFromTarget = 0;

    // public Shooter(int shootingMotorLeftCanId, int shootingMotorRightCanId, int rotatingMotorCanId, int rotatingEncoderCanId, int intakeMotorCanId, int LimitSwitchPort) {
    //     m_shootingMotorLeft = new CANSparkMax(shootingMotorLeftCanId, MotorType.kBrushless);
    //     m_shootingMotorRight = new CANSparkMax(shootingMotorRightCanId, MotorType.kBrushless);
    //     m_rotatingMotor = new CANSparkMax(rotatingMotorCanId, MotorType.kBrushless);
    //     m_intakeMotor = new CANSparkMax(intakeMotorCanId, MotorType.kBrushless);
    //     m_rotatingEncoder = new CANcoder(rotatingEncoderCanId);
    //     m_rotatingController = new PIDController(ShooterConstants.kShootingRotatingP, ShooterConstants.kShootingRotatingI, ShooterConstants.kShootingRotatingD);
    //     m_rotatingController.setTolerance(ShooterConstants.kShooterTolerance);
    //     m_limitSwitch = new DigitalInput(LimitSwitchPort);
    //     m_feedForward = new ArmFeedforward(rotatingEncoderCanId, intakeMotorCanId, LimitSwitchPort);
    // }

    /**
     * Constructor that uses constants defined in Constants.ShooterConstants
     */
    public Shooter() {
        m_shootingMotorLeft = new CANSparkMax(ShooterConstants.kShootingLeftCanId, MotorType.kBrushless);
        m_shootingMotorRight = new CANSparkMax(ShooterConstants.kShootingRightCanId, MotorType.kBrushless);
        m_rotatingMotor = new CANSparkMax(ShooterConstants.kShootingRotatingCanId, MotorType.kBrushless);
        m_intakeMotor = new CANSparkMax(ShooterConstants.kShooterIntakeCanId, MotorType.kBrushless);
        m_rotatingEncoder = new CANcoder(ShooterConstants.kShooterRotatingEncoderCanId);
        m_rotatingController = new PIDController(ShooterConstants.kShootingRotatingP, ShooterConstants.kShootingRotatingI, ShooterConstants.kShootingRotatingD);
        m_rotatingController.setTolerance(ShooterConstants.kShooterTolerance);
        m_limitSwitch = new DigitalInput(ShooterConstants.kLimitSwitchPort);
    }

    public double getAngleRadians() {
        return m_rotatingEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI * ShooterConstants.kShooterEncoderRatio;
    }

    public double getAngleDegrees() {
        return m_rotatingEncoder.getAbsolutePosition().getValueAsDouble() * 360 * ShooterConstants.kShooterEncoderRatio;
    }

    private void setAngle(double targetAngle) {
        if (targetAngle > Math.PI/2) {
            targetAngle = Math.PI/2;
        }
        double cosineScalar = Math.cos(getAngleRadians());
        m_rotatingMotor.set(m_rotatingController.calculate(targetAngle, getAngleRadians()) + ShooterConstants.kShootingKG * cosineScalar);
    }

    private double calculateAngle(double distanceFromTarget) {
        // can -90 degrees to 90 degrees, but should only return ~15-45 degrees
        return Math.atan((FieldConstants.kSpeakerHeight-RobotConstants.kShooterHeight)/distanceFromTarget) * 180/Math.PI;
    }

    /**
     * Sets the current running command for the shooter
     * @param command What command to run. Should be ShooterConstants.(kLoadShooter, kFireShooter, kDefault). If setting command to kFireShooter, make sure that the robot is pointed at the target.
     */
    public void setCommand(String command) {
        m_currentCommand = command;
        m_timer.stop();
        m_timer.reset();
        m_timer2.stop();
        m_timer2.reset();
    }

    /**
     * Sets the curent dis from target. Should be continually updated whenever possible.
     * @param dis The distance from the speaker in meters
     */
    public void setDisFromTarget(double dis) {
        m_disFromTarget = dis;
    }

    /**
     * Runs the shooter. This function should be called every loop. Use the setCommand function and the setDistance for shooting and loading the shooter
     */
    public void periodic() {
        if (m_currentCommand == ShooterConstants.kFireShooterSpeaker) {
            m_timer.start();
            setAngle(calculateAngle(m_disFromTarget));
            m_shootingMotorLeft.set(1);
            m_shootingMotorRight.set(-1);
            m_intakeMotor.set(0);
            if (m_timer.get() > 4) {
                m_timer2.start();
                m_intakeMotor.set(.2);
                if (m_timer2.get() > 2) {
                    setCommand(ShooterConstants.kRest);
                }
            }
        } else if (m_currentCommand == ShooterConstants.kLoadShooter) {
            m_shootingMotorLeft.set(0);
            m_shootingMotorRight.set(0);
            setAngle(ShooterConstants.kLoadShooterPosition);
            if (!m_limitSwitch.get()) {
                m_intakeMotor.set(.2);
            } else {
                m_intakeMotor.set(0);
                setCommand(ShooterConstants.kRest);
                // m_intakeMotor.set(.2);
                // setCommand("loadShooter2");
            }
        } // else if (m_currentCommand == "loadShooter2") {
        //     m_shootingMotorLeft.set(0);
        //     m_shootingMotorRight.set(0);
        //     setAngle(ShooterConstants.kLoadShooterPosition);
        //     m_timer.start();
        //     if (m_timer.get() < 2) {
        //         m_intakeMotor.
        //     }
        // }
        else if (m_currentCommand == ShooterConstants.kClimbShooter) {
            m_timer.stop();
            m_timer.reset();
            m_timer2.stop();
            m_timer2.reset();
            m_shootingMotorLeft.set(0);
            m_shootingMotorRight.set(0);
            m_intakeMotor.set(0);
            setAngle(ShooterConstants.kClimbingShooterPosition);
        } else {
            m_timer.stop();
            m_timer.reset();
            m_timer2.stop();
            m_timer2.reset();
            m_shootingMotorLeft.set(0);
            m_shootingMotorRight.set(0);
            m_intakeMotor.set(0);
            setAngle(ShooterConstants.kLoadShooterPosition);
        }
    }
}
