package org.team100.frc2023.autonomous;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;

import org.team100.frc2023.commands.SwerveControllerCommand;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.sensors.RedundantGyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;

public class VasiliWaypointTrajectory extends Command {
    private final SwerveControllerCommand m_swerveController;

    public VasiliWaypointTrajectory(
            SwerveDriveSubsystem m_robotDrive,
            SwerveDriveKinematics kinematics,
            Supplier<Rotation2d> desiredRotation,
            RedundantGyro gyro,
            String path) {

        Trajectory trajectory = genTrajectory(path);

        m_swerveController = new SwerveControllerCommand(
                m_robotDrive,
                trajectory,
                desiredRotation);
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize() {
        m_swerveController.initialize();
    }

    @Override
    public void execute() {
        m_swerveController.execute();
        running.set(5);
    }

    @Override
    public boolean isFinished() {
        return m_swerveController.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveController.end(interrupted);
        running.set(0);
    }

    //////////////////////////////////////////////////////////

    private Trajectory genTrajectory(String path) {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
            return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
        }
        return new Trajectory();
    }

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("waypoint");
    private final DoublePublisher running = table.getDoubleTopic("running").publish();
}