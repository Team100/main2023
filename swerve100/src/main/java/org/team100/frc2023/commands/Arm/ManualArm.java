package org.team100.frc2023.commands.Arm;

import java.util.function.Supplier;

import org.team100.frc2023.subsystems.arm.ArmSubsystem;
import org.team100.lib.subsystems.arm.ArmAngles;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Manual arm control in joint coordinates. */
public class ManualArm extends CommandBase {
    public static class Config {
        public double speedRadS = 0.5;
    }

    private final Config m_config = new Config();
    private final ArmSubsystem m_arm;
    private final Supplier<Double> m_upperSpeed;
    private final Supplier<Double> m_lowerSpeed;

    public ManualArm(ArmSubsystem arm, Supplier<Double> lowerSpeed, Supplier<Double> upperSpeed) {
        m_arm = arm;
        m_lowerSpeed = lowerSpeed;
        m_upperSpeed = upperSpeed;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_arm.setControlNormal();
    }

    @Override
    public void execute() {
        ArmAngles measurement = m_arm.getMeasurement();
        ArmAngles reference = new ArmAngles(
                measurement.th1 + 0.02 * m_config.speedRadS * m_lowerSpeed.get(),
                measurement.th2 + 0.02 * m_config.speedRadS * m_upperSpeed.get());
        m_arm.setReference(reference);
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.setReference(m_arm.getMeasurement());
    }
}
