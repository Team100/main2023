package org.team100.frc2023.commands.Arm;

import org.team100.frc2023.subsystems.SwerveDriveSubsystem;
import org.team100.frc2023.subsystems.arm.ArmController;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetConeMode extends CommandBase {
    private final ArmController m_arm;
    private final SwerveDriveSubsystem m_robotDrive;

    public SetConeMode(ArmController arm, SwerveDriveSubsystem robotDrive) {
        m_arm = arm;
        m_robotDrive = robotDrive;
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        m_arm.cubeMode = false;
        m_robotDrive.indicator.yellow();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
