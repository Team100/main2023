package org.team100.frc2023.commands.Arm;

import org.team100.frc2023.subsystems.arm.ArmController;
import org.team100.lib.indicator.LEDIndicator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetCubeMode extends CommandBase {
    private final ArmController m_arm;
    private final LEDIndicator m_indicator;

    public SetCubeMode(ArmController arm, LEDIndicator indicator) {
        m_arm = arm;
        m_indicator = indicator;
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        m_arm.cubeMode = true;
        m_indicator.purple();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
