package frc.robot.subsystems.util;

import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.motors.AnalogTurningEncoder;
import frc.robot.subsystems.motors.FalconDriveEncoder;
import frc.robot.subsystems.motors.FalconDriveMotor;
import frc.robot.subsystems.motors.PWMTurningMotor;
import frc.robot.Constants.KModule;

/**
 * Sets up the modules we actually use, so that the drive and swerve systems
 * don't need to know these details and don't instantiate anything.
 */
public class SwerveModuleFactory {
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
            double turningOffset) {
        FalconDriveMotor driveMotor = new FalconDriveMotor(name, driveMotorCanId);
        double driveEncoderDistancePerTurn = KModule.kWheelDiameterMeters * Math.PI /
        KModule.kDriveReduction;
        FalconDriveEncoder driveEncoder = new FalconDriveEncoder(name,
                driveMotor,
                driveEncoderDistancePerTurn,
                driveEncoderReversed);
        PWMTurningMotor turningMotor = new PWMTurningMotor(name, turningMotorChannel);
        double turningGearRatio = 1.0; // andymark ma3 encoder is 1:1
        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(name,
                turningEncoderChannel, turningOffset,
                turningGearRatio, turningEncoderReversed);

        return new SwerveModule(name, driveMotor, turningMotor, driveEncoder, turningEncoder);
    }
}
