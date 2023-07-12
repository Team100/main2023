package org.team100.frc2023.commands.Manipulator;

import org.team100.frc2023.subsystems.Manipulator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

// TODO: obsolete?
public class Close extends CommandBase {
    Manipulator m_manipulator;
    boolean first = true;
    Timer m_timer;

    public Close(Manipulator manipulator) {
        m_manipulator = manipulator;
        m_timer = new Timer();
        addRequirements(m_manipulator);
    }

    @Override
    public void initialize() {
        m_manipulator.pinch.motor.configPeakCurrentLimit(45);
        m_timer.restart();
        first = true;
    }

    @Override
    public void execute() {
        if (m_manipulator.hasGamepiece() == false) {
            m_manipulator.pinch(-0.8);
        } else {
            m_manipulator.pinch.motor.configPeakCurrentLimit(7);
            m_manipulator.pinch(-0.2);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_manipulator.pinch(0);
        m_manipulator.pinch.motor.configPeakCurrentLimit(30);
    }
}
