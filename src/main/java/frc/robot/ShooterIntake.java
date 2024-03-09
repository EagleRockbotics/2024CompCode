package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;

public class ShooterIntake {

    public static Shooter m_shooter;
    public static Intake m_intake;
    private static Timer timer;

    /**
     * Constructor for subsystem, initializes a Shooter and Intake object
     */
    public ShooterIntake() {
        m_shooter = new Shooter();
        m_intake = new Intake();
        timer = new Timer();
    }
    
    /**
     * Runs the shooter and intake to load a note into the shooter
     */
    public void loadShot() {
        if (!m_intake.getLimitSwitch()) {
            m_intake.setAngle(ShooterIntakeConstants.kIntakeLowerPosition);
            m_intake.runIntake(1);
            m_shooter.setAngle(ShooterIntakeConstants.kShooterIntakePosition);
            m_shooter.runShooter(.1);
            m_shooter.runIntake(0);
        } else {
            m_intake.setAngle(ShooterIntakeConstants.kIntakeHigherPosition);
            m_shooter.setAngle(ShooterIntakeConstants.kShooterIntakePosition);
            m_shooter.runShooter(.1);
            if (m_intake.atSetPoint() && m_shooter.atSetPoint()) {
                m_intake.runPitchMotor(0);
                m_shooter.runRotatingMotor(0);
                timer.start();
                if (timer.get() < 3) {
                    m_shooter.runIntake(1);
                    m_intake.runIntake(-1);
                } else {
                    m_shooter.runIntake(0);
                    m_intake.runIntake(0);
                    timer.stop();
                    timer.reset();
                }
            }
        }
    }

    private double calculateShooterAngle(double disFromTarget) {
        double vsquared = Math.pow(ShooterIntakeConstants.kShooterVelocity, 2);
        double vfourth = Math.pow(ShooterIntakeConstants.kShooterVelocity, 4);
        double g = -9.8;
        double h1 = FieldConstants.kSpeakerHeightClose;
        double h2 = FieldConstants.kSpeakerHeightFar;
        double disFromCloseEdge = disFromTarget - FieldConstants.kCloseEdgeFromTag;
        double SquareRootFar = Math.sqrt(vfourth - ((g * Math.pow(disFromTarget, 2)) + (2 * h2 * vsquared)));
        double SquareRootClose = Math.sqrt(vfourth - ((g * Math.pow(disFromCloseEdge, 2)) + (2 * h1 * vsquared)));
        double thetaFar = Math.atan((vsquared + SquareRootFar) / (g * disFromTarget));
        double thetaClose = Math.atan((vsquared + SquareRootClose) / (g * disFromCloseEdge));
        return (thetaFar + thetaClose)/2;
    }

    /**
     * Tries to shoot a note into the speaker
     * @param disFromTarget the distance from the apriltag of the speaker
     * @param shoot whether or not to shoot
     */
    public void ShootStationary(double disFromTarget, boolean shoot) {
        SmartDashboard.putBoolean("Aimed", false);
        m_intake.setAngle(ShooterIntakeConstants.kIntakeHigherPosition);
        if (m_intake.atSetPoint()) {
            m_intake.runPitchMotor(0);
        }
        m_intake.runIntake(0);
        m_shooter.runShooter(1);
        double angle = calculateShooterAngle(disFromTarget);
        if (angle != Double.NaN) {
            m_shooter.setAngle(Rotation2d.fromRadians(angle));
            if (m_shooter.atSetPoint()) {
                SmartDashboard.putBoolean("Aimed", true);
            }
        }
        m_shooter.runIntake(0);
        if (shoot) {
            m_shooter.runIntake(1);
        }
    }

    public void ManualShooterControl(Rotation2d angle, boolean shoot) {
        m_shooter.setAngle(angle);
        m_intake.setAngle(ShooterIntakeConstants.kIntakeHigherPosition);
        m_intake.runIntake(0);
        m_shooter.runShooter(1);
        if (shoot) {
            m_shooter.runIntake(1);
        } else {
            m_shooter.runIntake(0);
        }
    }

    public void setClimb() {
        m_shooter.runIntake(0);
        m_shooter.runShooter(.1);
        m_intake.runIntake(0);
        m_intake.setAngle(ShooterIntakeConstants.kIntakeHigherPosition);
        m_shooter.setAngle(ShooterIntakeConstants.kShooterClimbPosition);
    }

    /**
     * Function to load a note for working with a set angle shooter
     */
    public void setAngleLoadShot() {
        if (!m_intake.getLimitSwitch()) {
            m_intake.setAngle(ShooterIntakeConstants.kIntakeLowerPosition);
            m_shooter.setAngle(ShooterIntakeConstants.kShooterIntakePosition);
            m_intake.runIntake(1);
            m_shooter.runShooter(.1);
            if (m_shooter.atSetPoint()) {
                m_shooter.runRotatingMotor(0);
            }
            if (m_intake.atSetPoint()) {
                m_intake.runPitchMotor(0);
            }
        } else {
            m_intake.setAngle(ShooterIntakeConstants.kIntakeHigherPosition);
            m_shooter.setAngle(ShooterIntakeConstants.kShooterIntakePosition);
            m_intake.runIntake(0);
            m_shooter.runShooter(.1);
            if (m_shooter.atSetPoint()) {
                m_shooter.runRotatingMotor(0);
            }
            if (m_intake.atSetPoint()) {
                m_intake.runPitchMotor(0);
            }
        }
    }

    /**
     * Function to shoot a note for a set angle shooter
     * @param shoot whether or not to shoot
     */
    public void setAngleShoot(boolean shoot) {
        m_intake.setAngle(ShooterIntakeConstants.kIntakeHigherPosition);
        m_shooter.setAngle(ShooterIntakeConstants.kShooterIntakePosition);
        m_shooter.runShooter(1);
        if (m_shooter.atSetPoint()) {
            m_shooter.runRotatingMotor(0);
        }
        if (m_intake.atSetPoint()) {
            m_intake.runPitchMotor(0);
        }
        if (shoot) {
            m_intake.runIntake(-1);
        } else {
            m_intake.runIntake(0);
        }
    }
}
