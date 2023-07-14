package org.team100.frc2023.autonomous;

import org.team100.frc2023.commands.AutoLevel;
import org.team100.frc2023.commands.Arm.ArmTrajectory;
import org.team100.frc2023.commands.Arm.SetCubeMode;
import org.team100.frc2023.commands.Manipulator.Eject;
import org.team100.frc2023.subsystems.Manipulator;
import org.team100.frc2023.subsystems.arm.ArmController;
import org.team100.frc2023.subsystems.arm.ArmPosition;
import org.team100.lib.indicator.LEDIndicator;
import org.team100.lib.sensors.RedundantGyro;
import org.team100.lib.controller.DriveControllers;
import org.team100.lib.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class VasiliAutonomous extends SequentialCommandGroup {
    ControlVectorList controlVectors = new ControlVectorList();

    public VasiliAutonomous(
            SwerveDriveSubsystem m_robotDrive,
            DriveControllers controllers,
            RedundantGyro m_gyro,
            ArmController m_arm,
            Manipulator m_manipulator,
            LEDIndicator indicator) {

        // Rotation2d desiredRots = new Rotation2d(Math.PI);
        // SwerveModuleState[] desiredStates = new SwerveModuleState[] {
        // new SwerveModuleState(0, desiredRots),
        // new SwerveModuleState(0, desiredRots),
        // new SwerveModuleState(0, desiredRots),
        // new SwerveModuleState(0, desiredRots)
        // };
        // CommandBase command = new CommandBase() {
        // @Override
        // public void initialize() {
        // m_robotDrive.setModuleStates(desiredStates);
        // };

        // };
        // command.addRequirements(m_robotDrive);

        controlVectors.add(new Spline.ControlVector(
                new double[] { m_robotDrive.getPose().getX(), 0, 0 },
                new double[] { m_robotDrive.getPose().getY(), 0, 0.0 }));

        controlVectors.add(new Spline.ControlVector(
                new double[] { 5.537, 0, 0 },
                new double[] { 4.922, 0, 0 }));

        controlVectors.add(new Spline.ControlVector(
                new double[] { 5.916, 0, 0 },
                new double[] { 2.656, 0, 0 }));

        addCommands(
                // TODO: do we need this?

                // moveFromStartingPoseToGamePiece
                // .newMoveFromStartingPoseToGamePiece(
                // m_robotDrive,
                // new Pose2d(
                // m_robotDrive.getPose().getX(),
                // m_robotDrive.getPose().getY(),
                // new Rotation2d(Math.PI / 2)),
                // new Pose2d(0, 0.92, new Rotation2d(Math.PI / 2)),
                // () -> new Rotation2d(Math.PI)),

                // new ResetRotation(m_robotDrive, new Rotation2d(Math.PI)),

                // VasiliWaypointTrajectory
                // .newMoveFromStartingPoseToGamePiece(
                // m_robotDrive,
                // controlVectors,
                // () -> new Rotation2d(Math.PI),
                // "output/GoToCube(x).wpilib.json"),

                // new WaitCommand(2),
                // new Rotate(m_robotDrive, 0),
                // new WaitCommand(2),
                // new Rotate(m_robotDrive, Math.PI),
                // new WaitCommand(0.5),
                // VasiliWaypointTrajectory
                // .newMoveFromStartingPoseToGamePiece(
                // m_robotDrive,
                // controlVectors,
                // () -> new Rotation2d(Math.PI),
                // "output/GoBackToStation(x).wpilib.json")
                // new Rotate(m_robotDrive, 0)

                new SetCubeMode(m_arm, indicator),
                new ParallelDeadlineGroup(
                        new WaitCommand(3),
                        new ArmTrajectory(ArmPosition.HIGH, m_arm)),
                new ParallelDeadlineGroup(
                        new WaitCommand(2),
                        new Eject(m_manipulator)),
                new ParallelDeadlineGroup(
                        new WaitCommand(2),
                        new ArmTrajectory(ArmPosition.SAFE, m_arm)),

                new VasiliWaypointTrajectory(
                        m_robotDrive,
                        controllers,
                        () -> new Rotation2d(Math.PI),
                        m_gyro,
                        "output/BlueLeftExit.wpilib.json"),

                new AutoLevel(true, m_robotDrive, m_gyro));
    }
}
