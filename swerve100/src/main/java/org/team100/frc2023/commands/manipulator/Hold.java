package org.team100.frc2023.commands.manipulator;

import org.team100.frc2023.subsystems.ManipulatorInterface;
import org.team100.frc2023.subsystems.arm.ArmInterface;

import edu.wpi.first.wpilibj2.command.Command;

public class Hold extends Command {
    ManipulatorInterface m_manipulator;
    ArmInterface m_arm;


    public Hold(ManipulatorInterface manipulator, ArmInterface arm) {
        m_manipulator = manipulator;
        m_arm = arm;
    }

    @Override
    public void execute() {

        if(!m_arm.getCubeMode()){
            m_manipulator.set(-0.2, 30);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_manipulator.set(0, 30);
    }

}
