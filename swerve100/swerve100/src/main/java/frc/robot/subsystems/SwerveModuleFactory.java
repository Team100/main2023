package frc.robot.subsystems;

/**
 * Sets up the modules we actually use, so that the drive and swerve systems
 * don't need to know these details and don't instantiate anything.
 */
public class SwerveModuleFactory {
    /**
     * This is for the first AndyMark base.
     */
    public static SwerveModule newSwerveModule(
            String name,
            int driveMotorCanId,
            int turningMotorCanId,
            boolean driveEncoderReversed,
            boolean turningEncoderReversed,
            int angleRange,
            int angleZero) {
        TalonSRXDriveMotor driveMotor = new TalonSRXDriveMotor(name, driveMotorCanId);
        double driveEncoderDistancePerTurn = SwerveModule.kWheelDiameterMeters * Math.PI /
                SwerveModule.kDriveReduction;
        TalonSRXDriveEncoder driveEncoder = new TalonSRXDriveEncoder(name, driveMotor,
                driveEncoderDistancePerTurn, driveEncoderReversed);

        TalonSRXTurningMotor turningMotor = new TalonSRXTurningMotor(name, turningMotorCanId);
        TalonSRXTurningEncoder turningEncoder = new TalonSRXTurningEncoder(name, turningMotor, angleRange,
                angleZero, turningEncoderReversed);

        return new SwerveModule(name, driveMotor, turningMotor, driveEncoder, turningEncoder);
    }

    /**
     * This is for the second AndyMark base.
     */
    public static SwerveModule newSwerveModuleWithFalconDriveAndAnalogSteeringEncoders(
            String name,
            int driveMotorCanId,
            int turningMotorChannel,
            int turningEncoderChannel,
            boolean driveEncoderReversed,
            boolean turningEncoderReversed,
            double turningOffset,
            double turningGearRatio) {
        FalconDriveMotor driveMotor = new FalconDriveMotor(name, driveMotorCanId);
        double driveEncoderDistancePerTurn = SwerveModule.kWheelDiameterMeters * Math.PI /
                SwerveModule.kDriveReduction;
        FalconDriveEncoder driveEncoder = new FalconDriveEncoder(name,
                driveMotor,
                driveEncoderDistancePerTurn,
                driveEncoderReversed);
        PWMTurningMotor turningMotor = new PWMTurningMotor(name, turningMotorChannel);
        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(name,
                turningEncoderChannel, turningOffset,
                turningGearRatio, turningEncoderReversed);

        return new SwerveModule(name, driveMotor, turningMotor, driveEncoder, turningEncoder);
    }

    /**
     * This would work for the first AndyMark base if we plugged the steering
     * encoders into the RIO.
     */
    public static SwerveModule newSwerveModuleWithAnalogSteeringEncoders(
            String name,
            int driveMotorCanId,
            int turningMotorCanId,
            int turningEncoderChannel,
            boolean driveEncoderReversed,
            boolean turningEncoderReversed,
            double turningOffset, double turningGearRatio) {
        TalonSRXDriveMotor driveMotor = new TalonSRXDriveMotor(name, driveMotorCanId);
        double driveEncoderDistancePerTurn = SwerveModule.kWheelDiameterMeters * Math.PI /
                SwerveModule.kDriveReduction;
        TalonSRXDriveEncoder driveEncoder = new TalonSRXDriveEncoder(name, driveMotor,
                driveEncoderDistancePerTurn, driveEncoderReversed);

        TalonSRXTurningMotor turningMotor = new TalonSRXTurningMotor(name, turningMotorCanId);
        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(name, turningEncoderChannel, turningOffset,
                turningGearRatio, turningEncoderReversed);

        return new SwerveModule(name, driveMotor, turningMotor, driveEncoder, turningEncoder);
    }

    /**
     * This is for the Swervo project. It is not ready for use.
     */
    public static SwerveModule newSwervoModule(
            String name,
            int driveMotorCanId,
            int turningMotorCanId,
            int turningEncoderChannel,
            boolean driveEncoderReversed,
            boolean turningEncoderReversed,
            double turningOffset, double turningGearRatio) {
        TalonSRXDriveMotor driveMotor = new TalonSRXDriveMotor(name, driveMotorCanId);
        double driveEncoderDistancePerTurn = SwerveModule.kWheelDiameterMeters * Math.PI /
                SwerveModule.kDriveReduction;
        TalonSRXDriveEncoder driveEncoder = new TalonSRXDriveEncoder(name, driveMotor,
                driveEncoderDistancePerTurn, driveEncoderReversed);

        TalonSRXTurningMotor turningMotor = new TalonSRXTurningMotor(name, turningMotorCanId);
        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(name, turningEncoderChannel, turningOffset,
                turningGearRatio, turningEncoderReversed);

        return new SwerveModule(name, driveMotor, turningMotor, driveEncoder, turningEncoder);
    }
}
