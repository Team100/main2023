package org.team100.frc2023.subsystems;

import org.team100.frc2023.subsystems.game_piece_detection.GamepieceLocator;
import org.team100.lib.motors.FRCTalonSRX;
import org.team100.lib.motors.FRCTalonSRX.FRCTalonSRXBuilder;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {
    private final FRCTalonSRX m_motor;
    private final GamepieceLocator gamepieceLocator;

    public Manipulator() {
        m_motor = new FRCTalonSRXBuilder(10)
                .withInverted(false)
                .withSensorPhase(false)
                .withPeakOutputForward(1)
                .withPeakOutputReverse(-1)
                .withNeutralMode(NeutralMode.Brake)
                .withCurrentLimitEnabled(true)
                .build();
        m_motor.motor.configPeakCurrentLimit(30);
        m_motor.motor.configPeakCurrentDuration(1000);
        gamepieceLocator = new GamepieceLocator();
        SmartDashboard.putData("Manipulator", this);
    }

    public void set(double speed1_1, int currentLimit) {
        m_motor.motor.configPeakCurrentLimit(currentLimit);
        m_motor.motor.set(speed1_1);
    }

    public double getStatorCurrent() {
        return m_motor.motor.getStatorCurrent();
    }

    public double getGamePieceOffset() {
        return gamepieceLocator.getOffsetMeters();
    }

    public boolean hasGamepiece() {
        return gamepieceLocator.hasGamepiece();
    }

    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Output Current", () -> m_motor.motor.getStatorCurrent(), null);
        builder.addDoubleProperty("Input Current", () -> m_motor.motor.getSupplyCurrent(), null);
    }
}
