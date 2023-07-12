package org.team100.frc2023.subsystems.arm;


import org.team100.lib.motors.FRCNEO;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: remove this class
public class ArmSegment extends SubsystemBase {

    private FRCNEO motor;

    public ArmSegment(FRCNEO motor, String name) {
        this.motor = motor;
        SmartDashboard.putData(name, this);
    }

    public void setMotor(double x) {
        this.motor.motor.set(x);
    }

    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        // builder.addDoubleProperty("Error", () -> m_controller.getPositionError(),
        // null);
        // builder.addDoubleProperty("Measurement", () -> getMeasurement(), null);
        // builder.addDoubleProperty("Setpoint", () ->
        // m_controller.getSetpoint().position, null);
        // builder.addDoubleProperty("Goal", () -> m_controller.getGoal().position,
        // null);
        // builder.addDoubleProperty("Output", () -> output, null);
        // builder.addDoubleProperty("Degrees", () -> getDegrees(), null);

    }
}