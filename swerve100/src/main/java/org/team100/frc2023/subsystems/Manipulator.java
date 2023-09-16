package org.team100.frc2023.subsystems;

import org.team100.lib.config.Identity;
import org.team100.lib.motor.FRCTalonSRX;
import org.team100.lib.motor.FRCTalonSRX.FRCTalonSRXBuilder;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Manipulator extends Subsystem implements ManipulatorInterface {
    private static class Noop extends Subsystem implements ManipulatorInterface {

        @Override
        public Subsystem subsystem() {
            return this;
        }

        @Override
        public void set(double speed1_1, int currentLimit) {
        }

        @Override
        public double getStatorCurrent() {
            return 0;
        }

       
    }

    public static class Factory {
        private final Identity m_identity;

        public Factory(Identity identity) {
            m_identity = identity;
        }

        public ManipulatorInterface get() {
            switch (m_identity) {
                case COMP_BOT:
                    return new Manipulator();
                default:
                    return new Noop();
            }
        }
    }

    private final FRCTalonSRX m_motor;
   
    private Manipulator() {
        m_motor = new FRCTalonSRXBuilder(10)
                .withInverted(false)
                .withSensorPhase(false)
                .withPeakOutputForward(1)
                .withPeakOutputReverse(-1)
                .withNeutralMode(NeutralMode.Brake)
                .withCurrentLimitEnabled(true)
                .build();
        m_motor.configPeakCurrentLimit(30);
        m_motor.configPeakCurrentDuration(1000);
        SmartDashboard.putData("Manipulator", this);
    }

    public void set(double speed1_1, int currentLimit) {
        m_motor.configPeakCurrentLimit(currentLimit);
        m_motor.set(speed1_1);
    }

    public double getStatorCurrent() {
        return m_motor.getStatorCurrent();
    }

    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Output Current", () -> m_motor.getStatorCurrent(), null);
        builder.addDoubleProperty("Input Current", () -> m_motor.getSupplyCurrent(), null);
    }

    @Override
    public Subsystem subsystem() {
        return this;
    }
}
