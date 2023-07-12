package org.team100.frc2023.subsystems;

import org.team100.frc2023.subsystems.game_piece_detection.GamepieceLocator;
import org.team100.lib.motors.FRCTalonSRX;
import org.team100.lib.motors.FRCTalonSRX.FRCTalonSRXBuilder;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {
    public FRCTalonSRX pinch;
    public AnalogEncoder position;
    public PIDController pinchController;
    private double origin;
    GamepieceLocator gamepieceLocator;

    public Manipulator() {
        pinch = new FRCTalonSRXBuilder(10)
                .withInverted(false)
                .withSensorPhase(false)
                .withPeakOutputForward(1)
                .withPeakOutputReverse(-1)
                .withNeutralMode(NeutralMode.Brake)
                .withCurrentLimitEnabled(true)
                .build();
        pinch.motor.configPeakCurrentLimit(30);
        pinch.motor.configPeakCurrentDuration(1000);
        gamepieceLocator = new GamepieceLocator();
        position = new AnalogEncoder(4);
        position.reset();
        pinchController = new PIDController(0.2, 0, 0);
        SmartDashboard.putData("Manipulator", this);
    }

    public double getOrigin() {
        return origin;
    }

    public void pinch(double d) {
        pinch.motor.set(d);

    }

    public void pinchv2(double x, double y) {
        if (x > 0) {
            pinch.drivePercentOutput(x / 4);
        } else if (y > 0) {
            pinch.drivePercentOutput(-y / 4);
        } else {
            pinch.drivePercentOutput(0);
        }

    }

    public double getStatorCurrent() {
        return pinch.motor.getStatorCurrent();
    }

    public boolean getInnerLimitSwitch() {
        return pinch.motor.isFwdLimitSwitchClosed() == 1;
    }

    public double getPosition() {
        return position.get();
    }

    public boolean getForwardLimitSwitch() {
        if (pinch.motor.isFwdLimitSwitchClosed() == 1) {
            return true;
        } else {
            return false;
        }
    }

    public boolean getReverseLimitSwitch() {
        if (pinch.motor.isRevLimitSwitchClosed() == 1) {
            return true;
        } else {
            return false;
        }
    }

    public double getGamePieceOffset() {
        return gamepieceLocator.getOffsetMeters();
    }

    public boolean hasGamepiece() {
        return gamepieceLocator.hasGamepiece();
    }

    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Position", () -> getPosition(), null);
        builder.addBooleanProperty("Fwd Limit Switch", () -> getForwardLimitSwitch(), null);
        builder.addBooleanProperty("Rev Limit Switch", () -> getReverseLimitSwitch(), null);
        builder.addDoubleProperty("Output Current", () -> pinch.motor.getStatorCurrent(), null);
        builder.addDoubleProperty("Input Current", () -> pinch.motor.getSupplyCurrent(), null);
    }
}
