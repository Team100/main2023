package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
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
                MathUtil.applyDeadband(m_control.xSpeed(), .1),
                MathUtil.applyDeadband(m_control.ySpeed(), .1),
                MathUtil.applyDeadband(m_control.rotSpeed(), .1),
                true);
    }
}
