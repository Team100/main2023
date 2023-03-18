package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveModuleFactory {

    public static SwerveModule WCPModule(
            String name,
            int driveMotorCanId,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset,
            double currentLimit) {
        final double kWheelDiameterMeters = 0.1015; // WCP 4 inch wheel
        final double kDriveReduction = 5.50; // see wcproducts.com, this is the "fast" ratio.
        final double driveEncoderDistancePerTurn = kWheelDiameterMeters * Math.PI / kDriveReduction;
        final double turningGearRatio = 1.0;

        FalconDriveMotor driveMotor = new FalconDriveMotor(name, driveMotorCanId, currentLimit);
        FalconDriveEncoder driveEncoder = new FalconDriveEncoder(name, driveMotor, driveEncoderDistancePerTurn);
        
        FalconTurningMotor turningMotor = new FalconTurningMotor(name, turningMotorCanId);

        // NeoTurningMotor turningMotor = new NeoTurningMotor(name, turningMotorCanId);
        
        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(name, turningEncoderChannel, turningOffset,
                turningGearRatio);

        // DRIVE PID
        PIDController driveController = new PIDController( //
                0.1, // kP
                0.3, // kI: nonzero I eliminates small errors, e.g. to finish rotations.
                0.0); // kD
        driveController.setIntegratorRange(-0.01, 0.01); // Note very low windup limit.

        // TURNING PID
        ProfiledPIDController turningController = new ProfiledPIDController(
                0.5, // kP: low P because not much reduction gearing.
                0.0, // kI
                0.0, // kD
                new TrapezoidProfile.Constraints( //
                        20 * Math.PI, // max angular speed radians/sec
                        20 * Math.PI)); // max accel radians/sec/sec
        turningController.enableContinuousInput(0, 2 * Math.PI);

        // DRIVE FF
        SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward( //
                0.05, // kS: from experiment; overcome friction for low-effort moves
                .17, // kV
                0);

        // TURNING FF
        SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward( //
                0.05, // kS: friction is unimportant
                0.003,// kV: from experiment; higher than AM modules, less reduction gear
                0); // kA: I have no idea what this value should be

        return new SwerveModule(name, driveMotor, turningMotor, driveEncoder, turningEncoder,
                driveController, turningController, driveFeedforward, turningFeedforward);
    }

    // for 8048's config
    public static SwerveModule AMCANModule(
            String name,
            int driveMotorCanId,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset,
            double currentLimit) {
        final double kWheelDiameterMeters = 0.1016; // AndyMark Swerve & Steer has 4 inch wheel
        final double kDriveReduction = 6.67; // see andymark.com/products/swerve-and-steer
        final double driveEncoderDistancePerTurn = kWheelDiameterMeters * Math.PI / kDriveReduction;
        final double turningGearRatio = 1.0; // andymark ma3 encoder is 1:1

        FalconDriveMotor driveMotor = new FalconDriveMotor(name, driveMotorCanId, currentLimit);
        FalconDriveEncoder driveEncoder = new FalconDriveEncoder(name, driveMotor, driveEncoderDistancePerTurn);
        CANTurningMotor turningMotor = new CANTurningMotor(name, turningMotorCanId);
        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(name, turningEncoderChannel, turningOffset,
                turningGearRatio);

        // DRIVE PID
        PIDController driveController = new PIDController( //
                0.1, // kP
                0, // kI
                0); // kD

        // TURNING PID
        ProfiledPIDController turningController = new ProfiledPIDController( //
                0.5, // kP
                0, // kI
                0, // kD
                new TrapezoidProfile.Constraints(
                        20 * Math.PI, // speed rad/s
                        20 * Math.PI)); // accel rad/s/s
        turningController.enableContinuousInput(0, 2 * Math.PI);

        // DRIVE FF
        SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward( //
                0.0, // kS
                .5); // kV

        // TURNING FF
        SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward( //
                0.1, // kS
                0.005); // kV

        return new SwerveModule(name, driveMotor, turningMotor, driveEncoder, turningEncoder,
                driveController, turningController, driveFeedforward, turningFeedforward);

    }

    public static SwerveModule AMModule(
            String name,
            int driveMotorCanId,
            int turningMotorChannel,
            int turningEncoderChannel,
            double turningOffset,
            double currentLimit) {
        final double kWheelDiameterMeters = 0.1016; // AndyMark Swerve & Steer has 4 inch wheel
        final double kDriveReduction = 6.67; // see andymark.com/products/swerve-and-steer
        final double driveEncoderDistancePerTurn = kWheelDiameterMeters * Math.PI / kDriveReduction;
        final double turningGearRatio = 1.0; // andymark ma3 encoder is 1:1
        FalconDriveMotor driveMotor = new FalconDriveMotor(name, driveMotorCanId, currentLimit);
        FalconDriveEncoder driveEncoder = new FalconDriveEncoder(name, driveMotor, driveEncoderDistancePerTurn);
        PWMTurningMotor turningMotor = new PWMTurningMotor(name, turningMotorChannel);
        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(name, turningEncoderChannel, turningOffset,
                turningGearRatio);

        // DRIVE PID
        PIDController driveController = new PIDController(//
                0.1, // kP
                0, // kI
                0);// kD

        // TURNING PID
        ProfiledPIDController turningController = new ProfiledPIDController(//
                0.5, // kP
                0, // kI
                0, // kD
                new TrapezoidProfile.Constraints(
                        20 * Math.PI, // speed rad/s
                        20 * Math.PI)); // accel rad/s/s
        turningController.enableContinuousInput(0, 2 * Math.PI);

        // Drive(IVE FF
        SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(//
                0.04, // kS
                0.18, // kV
                0.01); // kA

        // TURNING FF
        SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward(//
                0.05, // kS
                0.003, // kV
                0); // kA

        return new SwerveModule(name, driveMotor, turningMotor, driveEncoder, turningEncoder,
                driveController, turningController, driveFeedforward, turningFeedforward);
    }
}
