package org.team100.frc2023.commands.Arm;

import org.team100.frc2023.subsystems.arm.ArmController;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Oscillate extends CommandBase {
    private final ArmController m_arm;
    private boolean movingUp = false;

    public Oscillate(ArmController arm) {
        m_arm = arm;
    }

    @Override
    public void initialize() {
        if (m_arm.getUpperArm() < ArmController.coneSubVal) {
            movingUp = false;
        } else {
            movingUp = true;
        }
    }

    @Override
    public void execute() {
        if (m_arm.getUpperArm() < ArmController.coneSubVal - 0.025) {
            movingUp = true;
        }

        if (m_arm.getUpperArm() > ArmController.coneSubVal + 0.025) {
            movingUp = false;
        }

        if (movingUp) {
            m_arm.setUpperArm(0.1);
        } else {
            m_arm.setUpperArm(-0.15);
        }

    }

}
