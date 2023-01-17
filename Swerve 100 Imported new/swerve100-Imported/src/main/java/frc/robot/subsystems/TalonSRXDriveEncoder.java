// package frc.robot.subsystems;

// import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class TalonSRXDriveEncoder implements DriveEncoder {
//     // see docs.ctre-phoenix.com/en/stable/ch14_MCSensor.html
//     private static final int kCIMcoderPulsesPerTurn = 80;
//     private final TalonSRXDriveMotor m_motor;
//     private final double m_distancePerPulse;

//     public TalonSRXDriveEncoder(String name,
//             TalonSRXDriveMotor motor,
//             double distancePerTurn,
//             boolean reverseDirection) {
//         m_motor = motor;
//         m_distancePerPulse = distancePerTurn / kCIMcoderPulsesPerTurn;
//         m_motor.setSensorPhase(reverseDirection);
//         SmartDashboard.putData(String.format("Talon SRX Drive Encoder %s", name), this);
//     }

//     public double getRate() {
//         return m_motor.getSelectedSensorVelocity() * 100 * m_distancePerPulse;
//     }

//     public void reset() {
//         m_motor.setSelectedSensorPosition(0);
//     }

//     @Override
//     public void initSendable(SendableBuilder builder) {
//         builder.setSmartDashboardType("FalconDriveEncoder");
//         builder.addDoubleProperty("Speed", this::getRate, null);
//     }
// }