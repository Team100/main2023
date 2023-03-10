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
import frc.robot.commands.Arm.ArmTrajectory;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.Arm.ArmController;
import frc.robot.subsystems.Arm.ArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SanjanAutonomous extends SequentialCommandGroup {
  /** Creates a new autonomous. */
  public SanjanAutonomous(SwerveDriveSubsystem m_robotDrive, ArmController arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //TODO make an isFinished for arm trajectory
    addCommands(
        new ParallelDeadlineGroup(new WaitCommand(2), new ArmTrajectory(ArmPosition.HIGH, arm)),
        new WaitCommand(2),
        new ParallelRaceGroup(new ArmTrajectory(ArmPosition.SAFE, arm), new WaitCommand(1.5)),
        new DriveToWaypoint2( new Pose2d(5.2, 6.9, new Rotation2d()), 0.0, () -> GoalOffset.center, m_robotDrive)
    );
    

   }
 }
