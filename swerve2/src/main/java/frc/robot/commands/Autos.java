// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.KAuto;
import frc.robot.subsystems.SwerveDrivetrain;

public final class Autos {
    /** Example static factory for an autonomous command. */
    public static CommandBase getAutoCommand(SwerveDrivetrain subsystem) {
        return Commands.sequence(traj(subsystem));
    }

    public static SwerveControllerCommand traj(SwerveDrivetrain drive){
        TrajectoryConfig config =
                new TrajectoryConfig(
                        KAuto.kMaxSpeedMetersPerSecond,
                        KAuto.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(SwerveDrivetrain.kDriveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at```  ` the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),

                // Pass through these two interior waypoints, making an 's' curve path
                // List.of(new Translation2d(1, .5), new Translation2d(2, -.5)),
                
                // JK I'm just gonna go straight (This comment brought to you by GitHub Copilot)
                new ArrayList<Translation2d> (),
                
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0, 3, new Rotation2d(Math.PI)),
                config);

        var thetaController =
            new ProfiledPIDController(
                KAuto.kPThetaController, 0, 0, KAuto.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Consumer<SwerveModuleState[]> stateConsumer = drive::setModuleStates;
        Supplier<Pose2d> poseSupplier = drive::getPose;
        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(exampleTrajectory, 
                    poseSupplier,
                    SwerveDrivetrain.kDriveKinematics,
                    new PIDController(KAuto.kPXController, 0, 0),
                    new PIDController(KAuto.kPYController, 0, 0),
                    thetaController,
                    stateConsumer,
                    drive);

        // Reset odometry to the starting pose of the trajectory.
        //drive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand;
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
