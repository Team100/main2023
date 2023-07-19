package org.team100.frc2023.commands;

import java.util.function.Supplier;

import org.team100.frc2023.autonomous.HolonomicDriveController2;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;

// TODO: discard this class, use a holonomic trajectory instead, e.g. like PathPlanner Waypoints
public class SwerveControllerCommand {
    private final SwerveDriveSubsystem m_robotDrive;
    private final Timer m_timer;
    private final Trajectory m_trajectory;
    private final Supplier<Rotation2d> m_desiredRotation;
    private final HolonomicDriveController2 m_controller;

    public SwerveControllerCommand(
            SwerveDriveSubsystem robotDrive,
            HolonomicDriveController2 controller,
            Trajectory trajectory,
            Supplier<Rotation2d> rotationSupplier) {
        m_robotDrive = robotDrive;
        m_controller = controller;
        m_trajectory = trajectory;
        m_desiredRotation = rotationSupplier;
        m_timer = new Timer();
    }

    public void execute() {
        State desiredState = m_trajectory.sample(m_timer.get());

        Pose2d currentPose = m_robotDrive.getPose();

        Twist2d fieldRelativeTarget = m_controller.calculate(
                currentPose,
                desiredState,
                m_desiredRotation.get());

        m_robotDrive.driveInFieldCoords(fieldRelativeTarget);
    }

    public void initialize() {
        m_timer.restart();
    }

    public void end(boolean interrupted) {
        m_timer.stop();
    }

    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
}
