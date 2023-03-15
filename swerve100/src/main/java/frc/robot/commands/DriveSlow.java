package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;
import team100.control.DualXboxControl;

public class DriveSlow extends CommandBase {
    private final SwerveDriveSubsystem m_robotDrive;
    private final DualXboxControl m_control;

    public DriveSlow(SwerveDriveSubsystem robotDrive, DualXboxControl control) {
        m_robotDrive = robotDrive;
        m_control = control;
        addRequirements(m_robotDrive);
    }

    @Override
    public void execute() {
        m_robotDrive.driveSlow(
                m_control.xSpeed(),
                m_control.ySpeed(),
                m_control.rotSpeed(),
                true);
    }
}
