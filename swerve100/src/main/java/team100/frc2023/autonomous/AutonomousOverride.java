// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team100.frc2023.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team100.frc2023.commands.AutoLevel;
import team100.frc2023.commands.DriveMobility;
import team100.frc2023.commands.ResetRotation;
import team100.frc2023.commands.Arm.ArmTrajectory;
import team100.frc2023.commands.Arm.SetCubeMode;
import team100.frc2023.commands.Manipulator.Close;
import team100.frc2023.subsystems.AHRSClass;
import team100.frc2023.subsystems.Manipulator;
import team100.frc2023.subsystems.SwerveDriveSubsystem;
import team100.frc2023.subsystems.Arm.ArmController;
import team100.frc2023.subsystems.Arm.ArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousOverride extends SequentialCommandGroup {

double armExtendDelay = 1.5;
  double manipulatorRunDelay = 0.2;
  double armSafeDelay = 2;
  /** Creates a new AutonomousOverride. */
  public AutonomousOverride(SwerveDriveSubsystem m_robotDrive, ArmController m_arm, Manipulator m_manipulator, AHRSClass m_gyro, int routine, boolean isBlueAlliance    ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetCubeMode(m_arm, m_robotDrive),
        new ParallelDeadlineGroup(new WaitCommand(armExtendDelay), new ArmTrajectory(ArmPosition.HIGH, m_arm)),
        new ParallelDeadlineGroup(new WaitCommand(manipulatorRunDelay), new Close(m_manipulator)),
        new ParallelDeadlineGroup(new WaitCommand(armSafeDelay), new ArmTrajectory(ArmPosition.SAFE, m_arm)),
        //     // new WaitCommand(0.25),
        new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)),


        new DriveMobility(m_robotDrive),
        new ParallelDeadlineGroup(new WaitCommand(1), new DriveStop(m_robotDrive)),
        new DriveToThreshold(m_robotDrive),
        new AutoLevel(true, m_robotDrive, m_gyro)



    );
  }
}
