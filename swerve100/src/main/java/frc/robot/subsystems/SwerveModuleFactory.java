package frc.robot.subsystems;

/**
 * Sets up the modules we actually use, so that the drive and swerve systems
 * don't need to know these details and don't instantiate anything.
 */
public class SwerveModuleFactory {

    public static SwerveModule frontLeft(double turning_offset) {
        return newSwerveModule(
                "Front Left",
                11,
                // 0, // motor
                30, // motor
                2, // encoder
                false, // drive reverse
                false, // steer encoder reverse
                turning_offset);
    }

    public static SwerveModule frontRight(double turning_offset) {
        return newSwerveModule(
                "Front Right",
                12,
                // 2, // motor
                32, // motor
                0, // encoder
                false, // drive reverse
                false, // steer encoder reverse
                turning_offset);
    }

    public static SwerveModule rearLeft(double turning_offset) {
        return newSwerveModule(
                "Rear Left",
                21,
                // 1, // motor
                31, // motor
                3, // encoder
                false, // drive reverse
                false, // steer encoder reverse
                turning_offset);
    }

    public static SwerveModule rearRight(double turning_offset) {
        return newSwerveModule(
                "Rear Right",
                22,
                // 3, // motor
                33, // motor
                1, // encoder
                false, // drive reverse
                false, // steer encoder reverse
                turning_offset);
    }

    /*
     * POINT TO THE ONE YOU WANT
     */
    public static SwerveModule newSwerveModule(String name,
            int driveMotorCanId,
            int turningMotorCanId,
            int turningEncoderChannel,
            boolean driveEncoderReversed,
            boolean turningEncoderReversed,
            double turningOffset) {
        return newSwerveModuleWithFalconDriveAndNeoSteering(name, driveMotorCanId, turningMotorCanId,
                turningEncoderChannel, driveEncoderReversed, turningEncoderReversed, turningOffset);

    }

    private static SwerveModule newSwerveModuleWithFalconDriveAndNeoSteering(
            String name,
            int driveMotorCanId,
            int turningMotorCanId,
            int turningEncoderChannel,
            boolean driveEncoderReversed,
            boolean turningEncoderReversed,
            double turningOffset) {
        FalconDriveMotor driveMotor = new FalconDriveMotor(name, driveMotorCanId);
        double driveEncoderDistancePerTurn = SwerveModule.kWheelDiameterMeters * Math.PI
                / SwerveModule.kDriveReduction;
        FalconDriveEncoder driveEncoder = new FalconDriveEncoder(name, driveMotor, driveEncoderDistancePerTurn,
                turningEncoderReversed);
        NeoTurningMotor turningMotor = new NeoTurningMotor(name, turningMotorCanId);
        double turningGearRatio = 1.0;
        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(name, turningEncoderChannel,
                turningOffset, turningGearRatio, turningEncoderReversed);
        return new SwerveModule(name, driveMotor, turningMotor, driveEncoder, turningEncoder);
    }

    /**
     * This is for the second AndyMark base.
     */
    private static SwerveModule newSwerveModuleWithFalconDriveAndAnalogSteeringEncoders(
            String name,
            int driveMotorCanId,
            int turningMotorChannel,
            int turningEncoderChannel,
            boolean driveEncoderReversed,
            boolean turningEncoderReversed,
            double turningOffset) {
        FalconDriveMotor driveMotor = new FalconDriveMotor(name, driveMotorCanId);
        double driveEncoderDistancePerTurn = SwerveModule.kWheelDiameterMeters * Math.PI /
                SwerveModule.kDriveReduction;
        FalconDriveEncoder driveEncoder = new FalconDriveEncoder(name, driveMotor, driveEncoderDistancePerTurn,
                driveEncoderReversed);
        PWMTurningMotor turningMotor = new PWMTurningMotor(name, turningMotorChannel);
        double turningGearRatio = 1.0; // andymark ma3 encoder is 1:1
        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(name,
                turningEncoderChannel, turningOffset,
                turningGearRatio, turningEncoderReversed);

        return new SwerveModule(name, driveMotor, turningMotor, driveEncoder, turningEncoder);
    }

}
