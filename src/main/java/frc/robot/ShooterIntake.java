package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;

public class ShooterIntake {

    private static Shooter m_shooter;
    private static Intake m_intake;

    /**
     * Constructor for subsystem, initializes a Shooter and Intake object
     */
    public ShooterIntake() {
        m_shooter = new Shooter();
        m_intake = new Intake();
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

    public void loadIntake() {
        SmartDashboard.putBoolean("Aimed", false);

        m_shooter.setAngle(ShooterIntakeConstants.kShooterIntakePosition);
        m_shooter.runShooter(0);
        m_shooter.runIntake(0);

        m_intake.setAngle(ShooterIntakeConstants.kIntakeLowerPosition);
        m_intake.runIntake(1);
    }

    public void loadShooter() {
        SmartDashboard.putBoolean("Aimed", false);

        m_shooter.setAngle(ShooterIntakeConstants.kShooterIntakePosition);
        m_shooter.runShooter(0);
        m_shooter.runIntake(1);

        m_intake.setAngle(ShooterIntakeConstants.kIntakeHigherPosition);
        if (m_intake.atSetPoint()) {
            m_intake.runIntake(-1);
        } else {
            m_intake.runIntake(0);
        }
    }

    public void idle() {
        SmartDashboard.putBoolean("Aimed", false);

        m_shooter.setAngle(ShooterIntakeConstants.kShooterIntakePosition);
        m_shooter.runShooter(0);
        m_shooter.runIntake(0);

        m_intake.setAngle(ShooterIntakeConstants.kIntakeHigherPosition);
        m_intake.runIntake(0);
    }

    public Rotation2d getShooterAngle() {
        return m_shooter.getAngle();
    }

    public Rotation2d getIntakeAngle() {
        return m_intake.getAngle();
    }

    /**
     * Runs the shooter with manual aim
     * @param degrees how many degrees to turn the shooter
     * @param shoot whether or not to shoot
     */
    public void ManualShooterControl(Rotation2d degrees, boolean shoot) {
        SmartDashboard.putBoolean("Aimed", false);
        Rotation2d angle = getShooterAngle().rotateBy(degrees);
        if (angle.getDegrees() < ShooterIntakeConstants.kShooterIntakePosition.getDegrees()) {
            angle = ShooterIntakeConstants.kShooterIntakePosition;
        } else if (angle.getDegrees() > ShooterIntakeConstants.kShooterMaxPosition.getDegrees()) {
            angle = ShooterIntakeConstants.kShooterMaxPosition;
        }
        m_shooter.setAngle(angle);
        m_intake.setAngle(ShooterIntakeConstants.kIntakeHigherPosition);
        m_shooter.runShooter(1);
        if (shoot) {
            m_shooter.runIntake(1);
        } else {
            m_shooter.runIntake(0);
        }
    }

    public void AutoShoot(double dis, boolean shoot) {
        m_shooter.runShooter(1);
        m_intake.setAngle(ShooterIntakeConstants.kIntakeHigherPosition);
        m_intake.runIntake(0);

        double angle = calculateShooterAngle(dis);
        if (angle > ShooterIntakeConstants.kShooterIntakePosition.getRadians() && angle < ShooterIntakeConstants.kShooterMaxPosition.getDegrees()) {
            m_shooter.setAngle(Rotation2d.fromRadians(angle));
            if (m_shooter.atSetPoint()) {
                SmartDashboard.putBoolean("Aimed", true);
            } else {
                SmartDashboard.putBoolean("Aimed", false);
            }
        } else {
            m_shooter.setAngle(ShooterIntakeConstants.kShooterIntakePosition);
            SmartDashboard.putBoolean("Aimed", false);
        }

        if (shoot) {
            m_shooter.runIntake(1);
        } else {
            m_shooter.runIntake(0);
        }
    }

    /**
     * Function to shoot a note for a set angle shooter
     * @param shoot whether or not to shoot
     */
    public void setAngleShoot(boolean shoot) {
        SmartDashboard.putBoolean("Aimed", false);
        m_intake.setAngle(ShooterIntakeConstants.kIntakeHigherPosition);
        m_shooter.setAngle(ShooterIntakeConstants.kShooterIntakePosition);
        m_shooter.runShooter(1);
        // if (m_shooter.atSetPoint()) {
        //     m_shooter.runRotatingMotor(0);
        // }
        // if (m_intake.atSetPoint()) {
        //     m_intake.runPitchMotor(0);
        // }
        if (shoot) {
            m_intake.runIntake(-1);
        } else {
            m_intake.runIntake(0);
        }
    }
}
