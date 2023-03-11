// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoLevel;
import frc.robot.commands.ResetRotation;
import frc.robot.commands.SelectGamePiece;
import frc.robot.commands.Arm.ArmTrajectory;
import frc.robot.commands.Arm.SetConeMode;
import frc.robot.commands.Arm.SetCubeMode;
import frc.robot.commands.Manipulator.Open;
import frc.robot.commands.Manipulator.Release;
import frc.robot.subsystems.AutonGamePiece;
import frc.robot.subsystems.AutonSelect;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.Arm.ArmController;
import frc.robot.subsystems.Arm.ArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Autonomous extends SequentialCommandGroup {
  /** Creates a new Autonomous. */
  public Autonomous(AutonSelect autonProcedure, AutonGamePiece gamePiece, SwerveDriveSubsystem m_robotDrive, ArmController arm, Manipulator m_manipulator) {
    
    if(autonProcedure == AutonSelect.BLUE2){
        addCommands(
            new SelectGamePiece(gamePiece, arm),
            new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)),
            new ParallelDeadlineGroup(new WaitCommand(2), new ArmTrajectory(ArmPosition.HIGH, arm)),
            // new WaitCommand(1),
            new ParallelDeadlineGroup(new WaitCommand(0.5), new Release(m_manipulator)),
            new ParallelDeadlineGroup(new WaitCommand(0.5), new Open(m_manipulator)),
            // new WaitCommand(2),
            new ParallelRaceGroup(new ArmTrajectory(ArmPosition.SAFE, arm), new WaitCommand(2)),
            new WaitCommand(0.25),
            new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)),
            new AutoLevel(false, m_robotDrive.m_gyro, m_robotDrive)
        );
    } else if(autonProcedure == AutonSelect.BLUE1){
        addCommands(
            new SelectGamePiece(gamePiece, arm),
            new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)),
            new WaitCommand(0.5),
            new ParallelDeadlineGroup(new WaitCommand(2), new ArmTrajectory(ArmPosition.HIGH, arm)),
            new WaitCommand(1),
            new ParallelDeadlineGroup(new WaitCommand(0.5), new Release(m_manipulator)),
            new ParallelDeadlineGroup(new WaitCommand(1), new Open(m_manipulator)),
            new WaitCommand(2),
            new ParallelRaceGroup(new ArmTrajectory(ArmPosition.SAFE, arm), new WaitCommand(1.5))


            // new WaitCommand(1),
            // new DriveToWaypoint3(new Pose2d(2.15, 4.70, Rotation2d.fromDegrees(-180)), 0, m_robotDrive),
            // new WaitCommand(2),
            // new DriveToWaypoint3( new Pose2d(5.56, 4.7, Rotation2d.fromDegrees(-180)), 0.0, m_robotDrive),
            // new WaitCommand(2),
            // new Rotate(m_robotDrive, 0),
            // new WaitCommand(2),
            // new DriveToWaypoint3( new Pose2d(5.56, 2.72, Rotation2d.fromDegrees(-180)), 0.0, m_robotDrive),
            // new WaitCommand(2),
            // new AutoLevel(true, m_robotDrive.m_gyro, m_robotDrive)
        );
    } else if(autonProcedure == AutonSelect.BLUE3){
        addCommands(
            new SelectGamePiece(gamePiece, arm),
            new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)),
            new WaitCommand(0.5),
            new ParallelDeadlineGroup(new WaitCommand(2), new ArmTrajectory(ArmPosition.HIGH, arm)),
            new WaitCommand(1),
            new ParallelDeadlineGroup(new WaitCommand(0.5), new Release(m_manipulator)),
            new ParallelDeadlineGroup(new WaitCommand(1), new Open(m_manipulator)),
            new WaitCommand(2),
            new ParallelRaceGroup(new ArmTrajectory(ArmPosition.SAFE, arm), new WaitCommand(1.5))
            // new WaitCommand(1),
            // new DriveToWaypoint3(new Pose2d(2.15, 0.754, Rotation2d.fromDegrees(-180)), 0, m_robotDrive),
            // new WaitCommand(2),
            // new DriveToWaypoint3( new Pose2d(5.56, 0.754, Rotation2d.fromDegrees(-180)), 0.0, m_robotDrive),
            // new WaitCommand(2),
            // new Rotate(m_robotDrive, 0),
            // new WaitCommand(2),
            // new DriveToWaypoint3( new Pose2d(5.56, 2.72, Rotation2d.fromDegrees(-180)), 0.0, m_robotDrive),
            // new WaitCommand(2),
            // new AutoLevel(true, m_robotDrive.m_gyro, m_robotDrive)
        );
    } 
    
  }
}
