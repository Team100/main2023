package org.team100.frc2023.commands.manipulator;

import org.team100.frc2023.subsystems.ManipulatorInterface;
import org.team100.frc2023.subsystems.arm.ArmInterface;

import edu.wpi.first.wpilibj2.command.Command;

public class Eject extends Command {
    ManipulatorInterface m_manipulator;
    ArmInterface m_arm;

    public Eject (ManipulatorInterface manipulator, ArmInterface arm) {
        m_manipulator = manipulator;
        m_arm = arm;
        addRequirements(m_manipulator.subsystem());
    }

    @Override
    public void initialize() {

        if(m_arm.getCubeMode()){
            m_manipulator.set(-0.8, 30);

        } else {
            m_manipulator.set(0.8, 30);

        }
    }

    @Override
    public void execute() {
    }
}
