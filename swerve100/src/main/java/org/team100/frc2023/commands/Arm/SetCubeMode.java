package org.team100.frc2023.commands.Arm;

import org.team100.frc2023.subsystems.SwerveDriveSubsystem;
import org.team100.frc2023.subsystems.arm.ArmController;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetCubeMode extends CommandBase {
    ArmController m_arm;
    SwerveDriveSubsystem m_robotDrive;
    boolean done;

    public SetCubeMode(ArmController arm, SwerveDriveSubsystem robotDrive) {
        m_arm = arm;
        m_robotDrive = robotDrive;
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        m_arm.cubeMode = true;
        m_robotDrive.visionDataProvider.indicator.cube();
        done = true;
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
