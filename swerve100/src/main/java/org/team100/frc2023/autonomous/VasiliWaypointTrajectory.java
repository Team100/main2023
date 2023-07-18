package org.team100.frc2023.autonomous;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;

import org.team100.frc2023.commands.SwerveControllerCommand;
import org.team100.lib.sensors.RedundantGyro;
import org.team100.lib.controller.DriveControllers;
import org.team100.lib.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VasiliWaypointTrajectory extends SwerveControllerCommand {
    private double isRunning = 5;

    public VasiliWaypointTrajectory(
            SwerveDriveSubsystem m_robotDrive,
            SwerveDriveKinematics kinematics,
            DriveControllers controllers,
            Supplier<Rotation2d> desiredRotation,
            RedundantGyro gyro,
            String path) {
        super(
                genTrajectory(path),
                m_robotDrive::getPose,
                kinematics,
                controllers,
                desiredRotation,
                m_robotDrive::setModuleStates,
                gyro,
                m_robotDrive);
        addRequirements(m_robotDrive);
        SmartDashboard.putData("Move From Starting Pose To Game Piece", this);
    }

    private static Trajectory genTrajectory(String path) {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
            return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
        }
        return new Trajectory();
    }

    @Override
    public void execute() {
        super.execute();
        isRunning = 5;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        isRunning = 0;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("IsRunning", () -> isRunning, null);
    }
}