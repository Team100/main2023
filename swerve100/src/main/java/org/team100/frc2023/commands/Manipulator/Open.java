package org.team100.frc2023.commands.Manipulator;

import org.team100.frc2023.subsystems.Manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;

// TODO: looks like obsolete?
public class Open extends CommandBase {
    private final Manipulator m_manipulator;
    double openPosition;

    public Open(Manipulator manipulator) {
        m_manipulator = manipulator;
        openPosition = -1.47;
        addRequirements(m_manipulator);
    }

    @Override
    public void execute() {
        // double openSpeed =
        // m_manipulator.pinchController.calculate(m_manipulator.getPosition(),
        // openPosition);
        // m_manipulator.pinch(openSpeed);

        m_manipulator.pinch(0.2);
    }

}
