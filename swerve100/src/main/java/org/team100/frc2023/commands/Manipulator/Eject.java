package org.team100.frc2023.commands.Manipulator;

import org.team100.frc2023.subsystems.Manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Eject extends CommandBase {
    private final Manipulator m_manipulator;

    public Eject(Manipulator manipulator) {
        m_manipulator = manipulator;
        addRequirements(m_manipulator);
    }

    @Override
    public void execute() {
        if (m_manipulator.hasGamepiece()) {
            m_manipulator.set(-0.2, 7);
        } else {
            m_manipulator.set(-0.8, 45);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_manipulator.set(0, 30);
    }
}
