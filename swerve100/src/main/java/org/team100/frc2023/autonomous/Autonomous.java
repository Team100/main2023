package org.team100.frc2023.autonomous;

import org.team100.frc2023.commands.AutoLevel;
import org.team100.frc2023.commands.DriveMobility;
import org.team100.frc2023.commands.Arm.ArmTrajectory;
import org.team100.frc2023.commands.Arm.SetCubeMode;
import org.team100.frc2023.commands.Manipulator.Eject;
import org.team100.frc2023.subsystems.AHRSClass;
import org.team100.frc2023.subsystems.Manipulator;
import org.team100.frc2023.subsystems.SwerveDriveSubsystem;
import org.team100.frc2023.subsystems.arm.ArmController;
import org.team100.frc2023.subsystems.arm.ArmPosition;
import org.team100.lib.autonomous.DriveStop;
import org.team100.lib.commands.ResetRotation;
import org.team100.lib.indicator.LEDIndicator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Autonomous extends SequentialCommandGroup {
    private static final double kArmExtendTimeout = 1.5;
    private static final double kManipulatorRunTimeout = 0.2;
    private static final double kArmSafeTimeout = 2;
    private static final double kStopTimeout = 1;

    private final SwerveDriveSubsystem m_robotDrive;
    private final ArmController m_arm;
    private final Manipulator m_manipulator;
    private final AHRSClass m_gyro;
    private final LEDIndicator m_indicator;

    // TODO: make routine an enum
    public Autonomous(
            SwerveDriveSubsystem robotDrive,
            ArmController arm,
            Manipulator manipulator,
            AHRSClass gyro,
            LEDIndicator indicator,
            int routine) {
        m_robotDrive = robotDrive;
        m_arm = arm;
        m_manipulator = manipulator;
        m_gyro = gyro;
        m_indicator = indicator;

        if (routine == 0) {
            placeCube();

        } else if (routine == 1) {
            placeCube();
            autoLevel(false);
            
        } else if (routine == 2) {
            placeCube();
            driveOutAndBack();
            autoLevel(true);
        }
    }

    private void placeCube() {
        addCommands(
                new SetCubeMode(m_arm, m_indicator),
                timeout(new ArmTrajectory(ArmPosition.HIGH, m_arm), kArmExtendTimeout),
                timeout(new Eject(m_manipulator), kManipulatorRunTimeout),
                timeout(new ArmTrajectory(ArmPosition.SAFE, m_arm), kArmSafeTimeout),
                new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)));
    }

    private void driveOutAndBack() {
        addCommands(
                new DriveMobility(m_robotDrive),
                timeout(new DriveStop(m_robotDrive), kStopTimeout),
                new DriveToThreshold(m_robotDrive));
    }

    private void autoLevel(boolean reversed) {
        addCommands(
                new AutoLevel(reversed, m_robotDrive, m_gyro));
    }

    // TODO: why do we need a timeout?
    private Command timeout(Command command, double seconds) {
        return new ParallelDeadlineGroup(new WaitCommand(seconds), command);
    }
}
