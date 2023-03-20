// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoLevel;
import frc.robot.commands.GoalOffset;
import frc.robot.commands.ResetRotation;
import frc.robot.commands.Arm.ArmTrajectory;
import frc.robot.commands.Arm.SetConeMode;
import frc.robot.commands.Arm.SetCubeMode;
import frc.robot.commands.Manipulator.Open;
import frc.robot.commands.Manipulator.Release;
import frc.robot.subsystems.AHRSClass;
import frc.robot.subsystems.AutonSelect;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.Arm.ArmController;
import frc.robot.subsystems.Arm.ArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SanjanAutonomous extends SequentialCommandGroup {
  /** Creates a new autonomous. */
  public SanjanAutonomous(AutonSelect autonProcedure, SwerveDriveSubsystem m_robotDrive, ArmController arm, Manipulator m_manipulator, AHRSClass m_gyro) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //TODO make an isFinished for arm trajectory
    //TODO make the measurments irl
    //TODO make charge station measurments

    if(autonProcedure == AutonSelect.RED1){ //origin is at right 
        addCommands(
            new SetConeMode(arm, m_robotDrive),
            new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)),
            new WaitCommand(0.5),
            new ParallelDeadlineGroup(new WaitCommand(2), new ArmTrajectory(ArmPosition.HIGH, arm)),
            new WaitCommand(1),
            new ParallelDeadlineGroup(new WaitCommand(0.5), new Release(m_manipulator)),
            new ParallelDeadlineGroup(new WaitCommand(1), new Open(m_manipulator)),
            new WaitCommand(2),
            new ParallelRaceGroup(new ArmTrajectory(ArmPosition.SAFE, arm), new WaitCommand(1.5)),
            new WaitCommand(1),
            new DriveToWaypoint3(new Pose2d(1.4, 6.9, Rotation2d.fromDegrees(-180)), 0, m_robotDrive, m_gyro),
            new WaitCommand(2),
            new DriveToWaypoint3( new Pose2d(5.2, 6.9, Rotation2d.fromDegrees(-180)), 0.0, m_robotDrive, m_gyro),
            new WaitCommand(2),
            new Rotate(m_robotDrive, 0),
            new WaitCommand(2),
            new DriveToWaypoint3( new Pose2d(5.2, 4.9, Rotation2d.fromDegrees(-180)), 0.0, m_robotDrive, m_gyro),
            new WaitCommand(2),
            new AutoLevel(true, m_robotDrive, m_gyro)
        );
    } else if (autonProcedure == AutonSelect.RED3){
        addCommands(
            new SetConeMode(arm, m_robotDrive),
            new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)),
            new WaitCommand(0.5),
            new ParallelDeadlineGroup(new WaitCommand(2), new ArmTrajectory(ArmPosition.HIGH, arm)),
            new WaitCommand(1),
            new ParallelDeadlineGroup(new WaitCommand(0.5), new Release(m_manipulator)),
            new ParallelDeadlineGroup(new WaitCommand(1), new Open(m_manipulator)),
            new WaitCommand(2),
            new ParallelRaceGroup(new ArmTrajectory(ArmPosition.SAFE, arm), new WaitCommand(1.5)),
            new WaitCommand(1),
            new DriveToWaypoint3(new Pose2d(1.4, 6.9, Rotation2d.fromDegrees(-180)), 0, m_robotDrive, m_gyro), // place holder to get out of community
            new WaitCommand(2),
            new DriveToWaypoint3( new Pose2d(5.2, 6.9, Rotation2d.fromDegrees(-180)), 0.0, m_robotDrive, m_gyro), // place holder for corner point for community
            new WaitCommand(2),
            new Rotate(m_robotDrive, 0),
            new WaitCommand(2),
            new DriveToWaypoint3( new Pose2d(5.2, 4.9, Rotation2d.fromDegrees(-180)), 0.0, m_robotDrive, m_gyro), // place holder for final point in front of charge station
            new WaitCommand(2),
            new AutoLevel(true, m_robotDrive, m_gyro)
        );    
    } else if (autonProcedure == AutonSelect.RED2){
        addCommands(
            new SetCubeMode(arm, m_robotDrive),
            new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)),
            new WaitCommand(0.5),
            new ParallelDeadlineGroup(new WaitCommand(2), new ArmTrajectory(ArmPosition.HIGH, arm)),
            new WaitCommand(1),
            new ParallelDeadlineGroup(new WaitCommand(0.5), new Release(m_manipulator)),
            new ParallelDeadlineGroup(new WaitCommand(1), new Open(m_manipulator)),
            new WaitCommand(2),
            new ParallelRaceGroup(new ArmTrajectory(ArmPosition.SAFE, arm), new WaitCommand(1.5)),
            new WaitCommand(1),
            new AutoLevel(false, m_robotDrive, m_gyro)
        );    
    } else if (autonProcedure == AutonSelect.BLUE1){ //origin is at the right
        addCommands(
            new SetCubeMode(arm, m_robotDrive),
            new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)),
            // new WaitCommand(0.5),
            new ParallelDeadlineGroup(new WaitCommand(2), new ArmTrajectory(ArmPosition.HIGH, arm)),
            new WaitCommand(1),
            new ParallelDeadlineGroup(new WaitCommand(0.5), new Release(m_manipulator)),
            new ParallelDeadlineGroup(new WaitCommand(1), new Open(m_manipulator)),
            new WaitCommand(2),
            new ParallelRaceGroup(new ArmTrajectory(ArmPosition.SAFE, arm), new WaitCommand(3)),
            new WaitCommand(1),
            new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)),
            new AutoLevel(false, m_robotDrive, m_gyro)

            // new DriveToWaypoint3(new Pose2d(1.4, 4.9, Rotation2d.fromDegrees(-180)), 0, m_robotDrive), // place holder to get out of community
            // new WaitCommand(2),
            // new DriveToWaypoint3( new Pose2d(5.2, 4.9, Rotation2d.fromDegrees(-180)), 0.0, m_robotDrive), // place holder for corner point for community
            // new WaitCommand(2),
            // new Rotate(m_robotDrive, 0),
            // new WaitCommand(2),
            // new DriveToWaypoint3( new Pose2d(5.2, 4.9, Rotation2d.fromDegrees(-180)), 0.0, m_robotDrive), // place holder for final point in front of charge station
            // new WaitCommand(2),
            // new AutoLevel(true, m_robotDrive.m_gyro, m_robotDrive)
        );    
    } else if (autonProcedure == AutonSelect.BLUE3){ //origin is at the right
        addCommands(
            new SetConeMode(arm, m_robotDrive),
            new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)),
            new WaitCommand(0.5),
            new ParallelDeadlineGroup(new WaitCommand(2), new ArmTrajectory(ArmPosition.HIGH, arm)),
            new WaitCommand(1),
            new ParallelDeadlineGroup(new WaitCommand(0.5), new Release(m_manipulator)),
            new ParallelDeadlineGroup(new WaitCommand(1), new Open(m_manipulator)),
            new WaitCommand(2),
            new ParallelRaceGroup(new ArmTrajectory(ArmPosition.SAFE, arm), new WaitCommand(1.5)),
            new WaitCommand(1),
            new DriveToWaypoint3(new Pose2d(1.4, 4.9, Rotation2d.fromDegrees(-180)), 0, m_robotDrive, m_gyro), // place holder to get out of community
            new WaitCommand(2),
            new DriveToWaypoint3( new Pose2d(5.2, 4.9, Rotation2d.fromDegrees(-180)), 0.0, m_robotDrive, m_gyro), // place holder for corner point for community
            new WaitCommand(2),
            new Rotate(m_robotDrive, 0),
            new WaitCommand(2),
            new DriveToWaypoint3( new Pose2d(5.2, 4.9, Rotation2d.fromDegrees(-180)), 0.0, m_robotDrive, m_gyro), // place holder for final point in front of charge station
            new WaitCommand(2),
            new AutoLevel(true, m_robotDrive, m_gyro)
        );    
    } else if (autonProcedure == AutonSelect.BLUE2){
        addCommands(
            new SetCubeMode(arm, m_robotDrive),
            new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)),
            new WaitCommand(0.5),
            new ParallelDeadlineGroup(new WaitCommand(2), new ArmTrajectory(ArmPosition.HIGH, arm)),
            new WaitCommand(1),
            new ParallelDeadlineGroup(new WaitCommand(0.5), new Release(m_manipulator)),
            new ParallelDeadlineGroup(new WaitCommand(1), new Open(m_manipulator)),
            new WaitCommand(2),
            new ParallelRaceGroup(new ArmTrajectory(ArmPosition.SAFE, arm), new WaitCommand(1.5)),
            new WaitCommand(1),
            new AutoLevel(false, m_robotDrive, m_gyro)
        );    
    }


    
    

   }
 }
