package org.team100.frc2023.commands.arm;

import org.team100.frc2023.subsystems.arm.ArmSubsystem;
import org.team100.lib.indicator.LEDIndicator;
import org.team100.lib.indicator.LEDIndicator.State;

import edu.wpi.first.wpilibj2.command.Command;

public class SetConeMode extends Command {
    private final ArmSubsystem m_arm;
    private final LEDIndicator m_indicator;

    public SetConeMode(ArmSubsystem arm, LEDIndicator indicator) {
        m_arm = arm;
        m_indicator = indicator;
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        m_arm.cubeMode = false;
        m_indicator.set(State.YELLOW);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
