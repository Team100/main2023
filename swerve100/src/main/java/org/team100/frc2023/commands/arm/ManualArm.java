package org.team100.frc2023.commands.arm;

import java.util.function.Supplier;

import org.team100.frc2023.subsystems.arm.ArmInterface;
import org.team100.lib.motion.arm.ArmAngles;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** Manual arm control in joint coordinates. */
public class ManualArm extends Command {
    public static class Config {
        public double maxSpeedRadS = 10;
    }

    private final Config m_config = new Config();
    private final ArmInterface m_arm;
    private final Supplier<Double> m_upperSpeed1_1;
    private final Supplier<Double> m_lowerSpeed1_1;
    ArmAngles m_reference;

    public ManualArm(ArmInterface arm, Supplier<Double> lowerSpeed1_1, Supplier<Double> upperSpeed1_1) {
        m_arm = arm;
        m_lowerSpeed1_1 = lowerSpeed1_1;
        m_upperSpeed1_1 = upperSpeed1_1;
        SmartDashboard.putData("Manual Arm", this);
        m_reference = new ArmAngles();
        addRequirements(arm.subsystem());
    }

    @Override
    public void initialize() {
        // m_arm.setReference(m_arm.getMeasurement());
    }

    @Override
    public void execute() {
        final double dt = 0.02;
        ArmAngles measurement = m_arm.getMeasurement();
        ArmAngles reference = new ArmAngles(
                measurement.th1 + dt * m_config.maxSpeedRadS * m_lowerSpeed1_1.get(),
                measurement.th2 + dt * m_config.maxSpeedRadS * m_upperSpeed1_1.get());
        m_reference = new ArmAngles(reference.th1, reference.th2);
        m_arm.setReference(reference);
    }

    @Override
    public void end(boolean interrupted) {
        // m_arm.setReference(m_arm.getMeasurement());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Lower Speed", () -> m_lowerSpeed1_1.get(), null);
        builder.addDoubleProperty("Upper Speed", () -> m_upperSpeed1_1.get(), null);
       
        // builder.addDoubleProperty("Reference Th1", () -> m_reference.th1, null);
        // builder.addDoubleProperty("Reference Th2", () -> m_reference.th2, null);
       


    }
}
