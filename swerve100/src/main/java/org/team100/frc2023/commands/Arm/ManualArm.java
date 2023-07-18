package org.team100.frc2023.commands.Arm;

import java.util.function.Supplier;

import org.team100.frc2023.subsystems.arm.ArmSubsystem;
import org.team100.lib.subsystems.arm.ArmAngles;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Manual arm control in joint coordinates. */
public class ManualArm extends CommandBase {
    public static class Config {
        public double maxSpeedRadS = 0.5;
    }

    private final Config m_config = new Config();
    private final ArmSubsystem m_arm;
    private final Supplier<Double> m_upperSpeed1_1;
    private final Supplier<Double> m_lowerSpeed1_1;

    public ManualArm(ArmSubsystem arm, Supplier<Double> lowerSpeed1_1, Supplier<Double> upperSpeed1_1) {
        m_arm = arm;
        m_lowerSpeed1_1 = lowerSpeed1_1;
        m_upperSpeed1_1 = upperSpeed1_1;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_arm.setControlNormal();
    }

    @Override
    public void execute() {
        final double dt = 0.02;
        ArmAngles measurement = m_arm.getMeasurement();
        ArmAngles reference = new ArmAngles(
                measurement.th1 + dt * m_config.maxSpeedRadS * m_lowerSpeed1_1.get(),
                measurement.th2 + dt * m_config.maxSpeedRadS * m_upperSpeed1_1.get());
        m_arm.setReference(reference);
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.setReference(m_arm.getMeasurement());
    }
}
