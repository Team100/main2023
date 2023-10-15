package org.team100.lib.motor.turning;

import org.team100.lib.encoder.turning.AnalogTurningEncoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
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
    private final double m_gearRatio = 355/6;
    private final WPI_TalonSRX m_motor;
    private final double ticksPerRevolution = 28;
    // private final int channel;
    private final int canID;
    public static final double kTurningCurrentLimit = 7;
    private final AnalogTurningEncoder m_encoder;

    public CANTurningMotor(String name, int channel, AnalogTurningEncoder encoder, int swerveBot) {
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
        m_motor.configNominalOutputForward(0);
        m_motor.configNominalOutputReverse(0);
        m_motor.configPeakOutputForward(1);
        m_motor.configPeakOutputReverse(-1);
        m_motor.configAllowableClosedloopError(0, 0, 30);
        m_motor.config_kF(0, 0);
        m_motor.config_kP(0, .5);
        m_motor.config_kI(0, 0);
        m_motor.config_kD(0, 0);
        double absolutePosition = 0;
        m_motor.setSelectedSensorPosition(absolutePosition);
        m_motor.configVoltageCompSaturation(11);
        m_motor.enableVoltageCompensation(true);
        m_motor.setSensorPhase(true);
        if (swerveBot == 2 && name != "Rear Right") {m_motor.setInverted(true);}
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

    public void setPIDVelocity(double outputRadiansPerSec, double outputRadiansPerSecPerSec) {
        double revolutionsPerSec = outputRadiansPerSec/(2*Math.PI);
        double revolutionsPerSec2 = outputRadiansPerSecPerSec/(2*Math.PI);
        double revsPer100ms = revolutionsPerSec / 10;
        double ticksPer100ms = revsPer100ms * ticksPerRevolution;
        DemandType type = DemandType.ArbitraryFeedForward;
        double Kn = 0.1121212;
        double Kf = 0.6;
        double Ke = 0.068842;
        double Ks = 0.007576;
        double VSat = 11;
        double kFF = (Kn*revolutionsPerSec + Ks*Math.signum(revolutionsPerSec))*m_gearRatio/VSat;
        m_motor.set(ControlMode.Velocity, ticksPer100ms*m_gearRatio, type, kFF);
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
        builder.addDoubleProperty("Encoder Value", () -> m_motor.getSelectedSensorPosition()/(m_gearRatio*ticksPerRevolution), null);
        builder.addDoubleProperty("Velocity Value", () -> m_motor.getSelectedSensorVelocity()/(ticksPerRevolution*m_gearRatio)*10, null);
    }

}