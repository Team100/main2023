package org.team100.frc2023.autonomous;

import java.util.List;

import org.team100.frc2023.commands.SwerveControllerCommand;
import org.team100.lib.sensors.RedundantGyro;
import org.team100.lib.controller.DriveControllers;
import org.team100.lib.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

// TODO: do we need this?
public class Forward extends SwerveControllerCommand {
    public Forward(SwerveDriveSubsystem m_robotDrive, SwerveDriveKinematics kinematics,
            DriveControllers controllers, double x, RedundantGyro gyro) {
        super(
                genTrajectory(m_robotDrive, kinematics, x),
                m_robotDrive::getPose,
                kinematics,
                controllers,
                () -> new Rotation2d(),
                m_robotDrive::setModuleStates,
                gyro,
                m_robotDrive);
        addRequirements(m_robotDrive);
    }

    private static Trajectory genTrajectory(SwerveDriveSubsystem m_robotDrive, SwerveDriveKinematics kinematics,
            double x) {
        Pose2d currentRobotPose = m_robotDrive.getPose();
        double xRobot = currentRobotPose.getX();
        double yRobot = currentRobotPose.getY();
        Rotation2d rotRobot = currentRobotPose.getRotation();
        if (x < 0) {
            rotRobot = new Rotation2d(rotRobot.getRadians() - Math.PI);
        }

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(4, 3).setKinematics(kinematics);

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(xRobot, yRobot, rotRobot),
                List.of(),
                new Pose2d(xRobot + x, yRobot, rotRobot),
                trajectoryConfig);
        return exampleTrajectory;
    }

}
