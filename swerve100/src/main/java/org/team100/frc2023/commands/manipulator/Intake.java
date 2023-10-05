package org.team100.frc2023.commands.manipulator;

import org.team100.frc2023.subsystems.ManipulatorInterface;
import org.team100.frc2023.subsystems.arm.ArmInterface;

import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends Command {
    private final ManipulatorInterface m_manipulator;
    private final ArmInterface m_arm;


    public Intake(ManipulatorInterface manipulator, ArmInterface arm) {
        m_manipulator = manipulator;
        m_arm = arm;
        addRequirements(m_manipulator.subsystem());
    }

    @Override
    public void execute() {

        if(!m_arm.getCubeMode()){
            m_manipulator.set(-0.8, 45);
        } else {
            m_manipulator.set(0.8, 45);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_manipulator.set(0, 30);
    }
}
