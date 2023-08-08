package org.team100.lib.motor.turning;

import org.team100.lib.encoder.turning.AnalogTurningEncoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CANTurningMotor implements TurningMotor, Sendable {
    // private final WPI_VictorSPX m_motor;
    private final WPI_TalonSRX m_motor;
    // private final int channel;
    private final int canID;
    public static final double kTurningCurrentLimit = 7;
    private final AnalogTurningEncoder m_encoder;

    public CANTurningMotor(String name, int channel, AnalogTurningEncoder encoder) {
        m_encoder = encoder;
        m_motor = new WPI_TalonSRX(channel);
        m_motor.configFactoryDefault();
        m_motor.setSelectedSensorPosition(0);
        m_motor.setNeutralMode(NeutralMode.Brake);
        m_motor.enableCurrentLimit(true);
        m_motor.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, kTurningCurrentLimit, kTurningCurrentLimit, 0));
        this.canID = channel;
        m_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        m_motor.setSensorPhase(true);
        m_motor.configNominalOutputForward(0);
        m_motor.configNominalOutputReverse(0);
        m_motor.configPeakOutputForward(1);
        m_motor.configPeakOutputReverse(-1);
        m_motor.configAllowableClosedloopError(0, 0, 30);
        m_motor.config_kF(0, .05);
        m_motor.config_kP(0, 0.05);
        m_motor.config_kI(0, 0);
        m_motor.config_kD(0, .25);
        double absolutePosition = 0;
        m_motor.setSelectedSensorPosition(absolutePosition);
        SmartDashboard.putData(String.format("CAN Turning Motor %s", name), this);
    }

    @Override   
    public double get() {
        return m_motor.get();
    }

    public WPI_TalonSRX getMotor() {
        return m_motor;
    }

    @Override
    public void set(double output) {
        m_motor.set(output);
    }

    public void setPIDVelocity(double outputRadiansPerSec) {
        double gearRatio = 71*40/48;
        double ticksPerRevolution = 1024;
        double revolutionsPerSec = outputRadiansPerSec/(2*Math.PI);
        double revsPer100ms = revolutionsPerSec/10;
        double ticksPer100ms = revsPer100ms*ticksPerRevolution;
        m_motor.set(ControlMode.Velocity, ticksPer100ms*gearRatio);
    }


    public void setPIDPosition(double outputRadians) {
        double ticksPerRevolution = 1024;
        double outputTicks = outputRadians / (2*Math.PI) * ticksPerRevolution;
        m_motor.set(ControlMode.Position, outputTicks);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("CANTurningMotor");
        builder.addDoubleProperty("Device ID", () -> canID, null);
        builder.addDoubleProperty("Output", this::get, null);
        builder.addDoubleProperty("Encoder Value", () -> m_motor.getSelectedSensorPosition(), null);
    }

}