package org.team100.frc2023.commands.arm;

import org.team100.frc2023.subsystems.arm.ArmSubsystem;
import org.team100.lib.indicator.LEDIndicator;
import org.team100.lib.indicator.LEDIndicator.State;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetCubeMode extends CommandBase {
    private final ArmSubsystem m_arm;
    private final LEDIndicator m_indicator;

    public SetCubeMode(ArmSubsystem arm, LEDIndicator indicator) {
        m_arm = arm;
        m_indicator = indicator;
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        m_arm.cubeMode = true;
        m_indicator.set(State.PURPLE);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
