package org.team100.frc2023.commands.Arm;

import org.team100.frc2023.control.Control;
import org.team100.frc2023.subsystems.arm.ArmController;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualArm extends CommandBase {
    private final ArmController arm;
    private final Control m_controller;

    public ManualArm(ArmController a, Control controller) {
        arm = a;
        m_controller = controller;
        addRequirements(arm);
    }

    @Override
    public void execute() {

        // if(arm.getLowerArm() <= arm.softStop && m_controller.getRightX()/4 < 0){
        // arm.driveManually(m_controller.getLeftY()/2, 0);
        // } else {

        arm.driveManually(m_controller.upperSpeed() / 4, m_controller.lowerSpeed() / 4);

        // }
    }
}
