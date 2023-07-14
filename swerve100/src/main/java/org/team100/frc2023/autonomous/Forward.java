package org.team100.frc2023.autonomous;

import java.util.List;

import org.team100.frc2023.commands.SwerveControllerCommand;
import org.team100.frc2023.subsystems.SwerveDriveSubsystem;
import org.team100.lib.subsystems.AHRSClass;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

// TODO: do we need this?
public class Forward extends SwerveControllerCommand {
    public Forward(SwerveDriveSubsystem m_robotDrive, double x, AHRSClass gyro) {
        super(
                genTrajectory(m_robotDrive, x),
                m_robotDrive::getPose,
                SwerveDriveSubsystem.kDriveKinematics,
                m_robotDrive.controllers.xController,
                m_robotDrive.controllers.yController,
                m_robotDrive.controllers.thetaController,
                () -> new Rotation2d(),
                m_robotDrive::setModuleStates,
                gyro,
                m_robotDrive);
        addRequirements(m_robotDrive);
    }

    private static Trajectory genTrajectory(SwerveDriveSubsystem m_robotDrive, double x) {
        Pose2d currentRobotPose = m_robotDrive.getPose();
        double xRobot = currentRobotPose.getX();
        double yRobot = currentRobotPose.getY();
        Rotation2d rotRobot = currentRobotPose.getRotation();
        if (x < 0) {
            rotRobot = new Rotation2d(rotRobot.getRadians() - Math.PI);
        }

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                4,
                3)
                .setKinematics(SwerveDriveSubsystem.kDriveKinematics);

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(xRobot, yRobot, rotRobot),
                List.of(),
                new Pose2d(xRobot + x, yRobot, rotRobot),
                trajectoryConfig);
        return exampleTrajectory;
    }

}
