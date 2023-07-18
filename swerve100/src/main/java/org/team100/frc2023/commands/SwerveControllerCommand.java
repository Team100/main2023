package org.team100.frc2023.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.team100.frc2023.autonomous.HolonomicDriveController2;
import org.team100.lib.controller.DriveControllers;
import org.team100.lib.sensors.RedundantGyro;
import org.team100.lib.subsystems.VeeringCorrection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

// TODO: discard this class, use a holonomic trajectory instead, e.g. like PathPlanner Waypoints
public class SwerveControllerCommand extends CommandBase {
    private final Timer m_timer = new Timer();
    private final Trajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final SwerveDriveKinematics m_kinematics;
    private final HolonomicDriveController2 m_controller;
    private final Consumer<SwerveModuleState[]> m_outputModuleStates;
    private final Supplier<Rotation2d> m_desiredRotation;
    private final RedundantGyro m_gyro;
    private final VeeringCorrection m_veering;

    public SwerveControllerCommand(
            Trajectory trajectory,
            Supplier<Pose2d> pose,
            SwerveDriveKinematics kinematics,
            DriveControllers controllers,
            Supplier<Rotation2d> rotationSupplier,
            Consumer<SwerveModuleState[]> outputModuleStates,
            RedundantGyro gyro,
            Subsystem... requirements) {
        m_trajectory = trajectory;
        m_pose = pose;
        m_kinematics = kinematics;
        m_controller = new HolonomicDriveController2(controllers.xController, controllers.yController, controllers.thetaController, gyro);
        m_gyro = gyro;
        m_veering = new VeeringCorrection(m_gyro);
        m_desiredRotation = rotationSupplier;
        m_outputModuleStates = outputModuleStates;
        addRequirements(requirements);
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();
        var desiredState = m_trajectory.sample(curTime);
        Rotation2d rotation2 = m_veering.correct(m_desiredRotation.get());
        ChassisSpeeds targetChassisSpeeds = m_controller.calculate(m_pose.get(), desiredState, rotation2);
        SwerveModuleState[] targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);
        m_outputModuleStates.accept(targetModuleStates);
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
}
