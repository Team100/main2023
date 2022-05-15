package ctre_shims;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

/** A class to read encoder data from CTRE motors, frc::Encoder compatible. */
public class TalonAngle implements Sendable, AutoCloseable {
  private final TalonSRX m_motor;
  private double m_inputRange = 1;
  private double m_inputOffset = 0;
  private final double m_outputRange = 2 * Math.PI;

  public TalonAngle(TalonSRX motor) {
    m_motor = motor;
    SendableRegistry.addLW(this, "Talon Encoder", motor.getDeviceID());
  }

  @Override
  public void close() {
    SendableRegistry.remove(this);
  }

  /**
   * Gets the current count. Returns the current count on the Encoder. This method compensates for
   * the decoding type.
   *
   * @return Current count from the Encoder adjusted for the 1x, 2x, or 4x scale factor.
   */
  public int get() {
    return (int) m_motor.getSelectedSensorPosition();
  }

  
  public double getAnalogIn() {
    return (double) m_motor.getSensorCollection().getAnalogIn();
  }

  public double getAnalogInRaw() {
    return (double) m_motor.getSensorCollection().getAnalogInRaw();
  }

  /** Reset the Encoder distance to zero. Resets the current count to zero on the encoder. */
  public void reset() {
    m_motor.setSelectedSensorPosition(0);
  }

  /**
   * Get the distance the robot has driven since the last reset as scaled by the value from {@link
   * #setDistancePerPulse(double)}.
   *
   * @return The distance driven since the last reset
   */
  public double getDistance() {
    return correctAndWrapDistance(get(), m_inputOffset, m_inputRange, m_outputRange);
  }

  /**
   * Correct for sensor rotation offset and range.  Out-of-bounds inputs just wrap.
   */
  public static double correctAndWrapDistance(
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

  public void setInputRange(double inputRange) {
    m_inputRange = inputRange;
  }

  public void setInputOffset(double inputOffset) {
    m_inputOffset = inputOffset;
  }

  /**
   * Set the direction sensing for this encoder. This sets the direction sensing on the encoder so
   * that it could count in the correct software direction regardless of the mounting.
   *
   * @param reverseDirection true if the encoder direction should be reversed
   */
  public void setReverseDirection(boolean reverseDirection) {
    m_motor.setSensorPhase(reverseDirection);
  }

  public FeedbackDevice getSelectedFeedbackSensor() {
    return FeedbackDevice.valueOf(m_motor.configGetParameter(ParamEnum.eFeedbackSensorType, 0));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType(
        String.format("Encoder (%s)", getSelectedFeedbackSensor().name()));
    builder.addDoubleProperty("Distance", this::getDistance, null);
    builder.addDoubleProperty("analog in", this::getAnalogIn, null);
    builder.addDoubleProperty("analog in raw", this::getAnalogInRaw, null);
  }
}