package org.team100.frc2023.commands.Manipulator;

import org.team100.frc2023.subsystems.Manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;

// TODO: obsolete?
public class CloseSlow extends CommandBase {
    Manipulator m_manipulator;

    public CloseSlow(Manipulator manipulator) {
        m_manipulator = manipulator;
    }

    @Override
    public void initialize() {
        m_manipulator.pinch.motor.configPeakCurrentLimit(10);
    }

    @Override
    public void execute() {
        m_manipulator.pinch(-0.2);
    }

    @Override
    public void end(boolean interrupted) {
        m_manipulator.pinch.motor.configPeakCurrentLimit(30);
        m_manipulator.pinch(0);
    }

}
