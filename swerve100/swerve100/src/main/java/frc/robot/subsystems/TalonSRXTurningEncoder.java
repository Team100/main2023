package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TalonSRXTurningEncoder implements TurningEncoder {
    private final TalonSRXTurningMotor m_motor;
    private final double m_inputRange;
    private final double m_inputOffset;
    private final double m_outputRange = 2 * Math.PI;

    public TalonSRXTurningEncoder(
            String name,
            TalonSRXTurningMotor motor,
            double inputRange,
            double inputOffset,
            boolean reverseDirection) {
        m_motor = motor;
        m_inputRange = inputRange;
        m_inputOffset = inputOffset;
        m_motor.setSensorPhase(reverseDirection);
        SmartDashboard.putData(String.format("Talon SRX Turning Encoder %s", name), this);
    }

    // TODO: pick one: get, analogin, or analogin raw; delete the others.
    private int get() {
        return (int) m_motor.getSelectedSensorPosition();
    }

    private double getAnalogIn() {
        return (double) m_motor.getSensorCollection().getAnalogIn();
    }

    private double getAnalogInRaw() {
        return (double) m_motor.getSensorCollection().getAnalogInRaw();
    }

    public void reset() {
        m_motor.setSelectedSensorPosition(0);
    }

    public double getAngle() {
        return correctAndWrapAngle(get(), m_inputOffset, m_inputRange, m_outputRange);
    }

    /**
     * Correct for sensor rotation offset and range. Out-of-bounds inputs wrap
     * around.
     */
    public static double correctAndWrapAngle(
            double inputPosition,
            double inputOffset,
            double inputRange,
            double outputRange) {
        double zeroedInput = inputPosition - inputOffset;
        double scaledZeroedInput = zeroedInput * outputRange / inputRange;
        if (scaledZeroedInput < 0) {
            return scaledZeroedInput + outputRange;
        }
        return scaledZeroedInput;
    }

    public void setReverseDirection(boolean reverseDirection) {
        m_motor.setSensorPhase(reverseDirection);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Talon SRX Turning Encoder");
        builder.addDoubleProperty("Angle", this::getAngle, null);
        builder.addDoubleProperty("Analog In", this::getAnalogIn, null);
        builder.addDoubleProperty("Analog In Raw", this::getAnalogInRaw, null);
    }
}
