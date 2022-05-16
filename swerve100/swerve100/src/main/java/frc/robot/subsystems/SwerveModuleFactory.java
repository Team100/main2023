package frc.robot.subsystems;

/**
 * Sets up the modules we actually use, so that the drive and swerve systems
 * don't need to know these details and don't instantiate anything.
 */
public class SwerveModuleFactory {
    public static SwerveModule newSwerveModule(
            String name,
            int driveMotorCanId,
            int turningMotorCanId,
            boolean driveEncoderReversed,
            boolean turningEncoderReversed,
            int angleRange,
            int angleZero) {
        TalonSRXDriveMotor driveMotor = new TalonSRXDriveMotor(name, driveMotorCanId);
        TalonSRXDriveEncoder driveEncoder = new TalonSRXDriveEncoder(name, driveMotor,
                SwerveModule.kDriveEncoderDistancePerPulse, driveEncoderReversed);

        TalonSRXTurningMotor turningMotor = new TalonSRXTurningMotor(name, turningMotorCanId);
        TalonSRXTurningEncoder turningEncoder = new TalonSRXTurningEncoder(name, turningMotor, angleRange,
                angleZero, turningEncoderReversed);

        return new SwerveModule(name, driveMotor, turningMotor, driveEncoder, turningEncoder);
    }

    public static SwerveModule newSwerveModuleWithAnalogSteeringEncoders(
            String name,
            int driveMotorCanId,
            int turningMotorCanId,
            int turningEncoderChannel,
            boolean driveEncoderReversed,
            boolean turningEncoderReversed,
            double turningOffset, double turningGearRatio) {
        TalonSRXDriveMotor driveMotor = new TalonSRXDriveMotor(name, driveMotorCanId);
        TalonSRXDriveEncoder driveEncoder = new TalonSRXDriveEncoder(name, driveMotor,
                SwerveModule.kDriveEncoderDistancePerPulse, driveEncoderReversed);

        TalonSRXTurningMotor turningMotor = new TalonSRXTurningMotor(name, turningMotorCanId);
        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(name, turningEncoderChannel, turningOffset,
                turningGearRatio, turningEncoderReversed);

        return new SwerveModule(name, driveMotor, turningMotor, driveEncoder, turningEncoder);
    }
}
