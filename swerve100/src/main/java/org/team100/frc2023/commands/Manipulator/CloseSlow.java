package org.team100.frc2023.commands.Manipulator;

import org.team100.frc2023.subsystems.Manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CloseSlow extends CommandBase {
    Manipulator m_manipulator;

    public CloseSlow(Manipulator manipulator) {
        m_manipulator = manipulator;
    }

    @Override
    public void execute() {
        m_manipulator.set(-0.2, 30);
    }

    @Override
    public void end(boolean interrupted) {
        m_manipulator.set(0, 30);
    }

}
