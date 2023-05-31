// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team100.frc2023.autonomous;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team100.frc2023.commands.SwerveControllerCommand;
import team100.frc2023.subsystems.AHRSClass;
import team100.frc2023.subsystems.SwerveDriveSubsystem;

/** Add your docs here. */
public class Circle extends CommandBase {
    AHRSClass m_gyro;
    SwerveDriveSubsystem m_robotDrive;
    double radius;
    boolean done;

    public Circle(SwerveDriveSubsystem m_robotDrive, double radius, AHRSClass gyro) {
        this.m_robotDrive = m_robotDrive;
        this.radius = radius;
        m_gyro = gyro;
        addRequirements(this.m_robotDrive);
    }

    @Override
    public void initialize() {
        SwerveControllerCommand s = new SwerveControllerCommand(
            genTrajectory(m_robotDrive, radius),
            m_robotDrive::getPose,
            SwerveDriveSubsystem.kDriveKinematics,
            m_robotDrive.xController,
            m_robotDrive.yController,
            m_robotDrive.thetaController,
            () -> new Rotation2d(),
            m_robotDrive::setModuleStates,
            m_gyro,
            m_robotDrive);

        CommandScheduler.getInstance().schedule(s);
        done = true;
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    // private static Trajectory genTrajectory(SwerveDriveSubsystem m_robotDrive) {
    //     double controlPointAngle = Math.atan2(
    //             (1.071626 - m_robotDrive.getPose().getY()),
    //             (14.513558 - m_robotDrive.getPose().getX())
    //     );
        
        // An example trajectory to follow. All units in meters.



private static Trajectory genTrajectory(SwerveDriveSubsystem m_robotDrive, double radius) {

        Pose2d currentRobotPose = m_robotDrive.getPose();
        double x = currentRobotPose.getX();
        double y = currentRobotPose.getY();

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                2,
                1)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(SwerveDriveSubsystem.kDriveKinematics);


        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(x, y, new Rotation2d(-Math.PI/2)),
                List.of(
                    new Translation2d(x + radius, y - radius),
                    new Translation2d(x + 2*radius, y),
                    new Translation2d(x + radius, y + radius)

                ),
                new Pose2d(x, y, new Rotation2d(-Math.PI/2)),
                trajectoryConfig
        );

        return exampleTrajectory;

}

}
