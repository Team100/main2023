package org.team100.frc2023.commands;

import org.team100.frc2023.control.Control;
import org.team100.frc2023.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

// TODO: do we need this?
public class DriveMedium extends CommandBase {

    private final SwerveDriveSubsystem m_robotDrive;
    private final Control m_control;

    public DriveMedium(SwerveDriveSubsystem robotDrive, Control control) {
        m_robotDrive = robotDrive;
        m_control = control;
        addRequirements(m_robotDrive);
    }

    @Override
    public void execute() {
        // m_robotDrive.driveMedium(
        // MathUtil.applyDeadband(m_control.xSpeed()/2, .02),
        // MathUtil.applyDeadband(m_control.ySpeed()/2, .02),
        // MathUtil.applyDeadband(m_control.rotSpeed(), .02),
        // true);

        double xVal = MathUtil.applyDeadband(m_control.xSpeed() / 2, .02);
        double yVal = MathUtil.applyDeadband(m_control.ySpeed() / 2, .02);
        xVal = Math.signum(xVal) * 1;
        yVal = Math.signum(yVal) * 1;
        m_robotDrive.drive(
                MathUtil.applyDeadband(xVal, .02),
                MathUtil.applyDeadband(yVal, .02),
                MathUtil.applyDeadband(m_control.rotSpeed(), .02),
                true);
    }
}
