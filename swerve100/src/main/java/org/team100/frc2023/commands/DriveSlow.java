package org.team100.frc2023.commands;

import org.team100.frc2023.control.Control;
import org.team100.frc2023.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveSlow extends CommandBase {
    private final SwerveDriveSubsystem m_robotDrive;
    private final Control m_control;

    public DriveSlow(SwerveDriveSubsystem robotDrive, Control control) {
        m_robotDrive = robotDrive;
        m_control = control;
        addRequirements(m_robotDrive);
    }

    @Override
    public void execute() {

        m_robotDrive.driveFine(
                MathUtil.applyDeadband(m_control.xSpeed()/2, .02),
                MathUtil.applyDeadband(m_control.ySpeed()/2, .02),
                MathUtil.applyDeadband(m_control.rotSpeed(), .02),
                true);
    }
}
