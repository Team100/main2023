package org.team100.frc2023.autonomous;

import org.team100.lib.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveConeWidth extends CommandBase {
    private static final double goalRotation = Math.PI;

    private final SwerveDriveSubsystem m_robotDrive;
    private final double m_coneWidth;
    private final ProfiledPIDController yController;
    private final PIDController m_rotationController;

    private double goalY = 0;

    public MoveConeWidth(SwerveDriveSubsystem robotDrive, double coneWidth) {
        m_robotDrive = robotDrive;
        m_coneWidth = coneWidth * 0.155;
        yController = new ProfiledPIDController(2, 0, 0, new Constraints(5, 0.7));
        yController.setTolerance(0.00000001);
        m_rotationController = new PIDController(2, 0, 0);
        m_rotationController.setTolerance(Math.PI / 180);
        m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize() {
        yController.reset(m_robotDrive.getPose().getY());
        goalY = m_robotDrive.getPose().getY() + m_coneWidth;
    }

    @Override
    public void execute() {
        double outputRot = m_rotationController.calculate(
                m_robotDrive.getPose().getRotation().getRadians(),
                goalRotation);
        double outputY = yController.calculate(m_robotDrive.getPose().getY(), goalY);
        m_robotDrive.driveMetersPerSec(new Twist2d(0, outputY, outputRot), true);
    }
}
