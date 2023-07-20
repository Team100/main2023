package org.team100.frc2023.commands.manipulator;

import org.team100.frc2023.subsystems.Manipulator;

import edu.wpi.first.wpilibj2.command.Command;

// TODO: looks like obsolete?
public class Open extends Command {
    private final Manipulator m_manipulator;

    public Open(Manipulator manipulator) {
        m_manipulator = manipulator;
        addRequirements(m_manipulator);
    }

    @Override
    public void execute() {
        m_manipulator.set(0.2, 30);
    }
}
