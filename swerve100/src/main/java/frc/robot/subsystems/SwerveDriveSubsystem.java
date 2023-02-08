package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.localization.VisionDataProvider;
import team100.config.Identity;

@SuppressWarnings("unused")
public class SwerveDriveSubsystem extends SubsystemBase {
    // TODO: make this an instance var
    public static final SwerveDriveKinematics kDriveKinematics;

    static {
        final double kTrackWidth;
        final double kWheelBase;
        switch (Identity.get()) {
            case SQUAREBOT:
                kTrackWidth = 0.650;
                kWheelBase = 0.650;
                break;
            case SWERVE_TWO:
                kTrackWidth = 0.380;
                kWheelBase = 0.445;
                break;
            case SWERVE_ONE:
                kTrackWidth = 0.449;
                kWheelBase = 0.464;
                break;
            default:
                throw new IllegalStateException("Identity is not swerve: " + Identity.get().name());
        }

        kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    }

    // TODO: what were these for?
    // public static final double ksVolts = 2;
    // public static final double kvVoltSecondsPerMeter = 2.0;
    // public static final double kaVoltSecondsSquaredPerMeter = 0.5;

    // SLOW SETTINGS
    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    // NOTE joel 2/8 used to be negative; inversions broken somewhere?
    public static final double kMaxAngularSpeedRadiansPerSecond = 5;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 10;

    // FAST SETTINGS. can the robot actually go this fast?
    // public static final double kMaxSpeedMetersPerSecond = 8;
    // public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    // public static final double kMaxAngularSpeedRadiansPerSecond = 10;
    // public static final double kMaxAngularSpeedRadiansPerSecondSquared = 100;

    private final SwerveModule m_frontLeft;
    private final SwerveModule m_frontRight;
    private final SwerveModule m_rearLeft;
    private final SwerveModule m_rearRight;

    // The gyro sensor. We have a Nav-X.
    public final AHRS m_gyro;
    // Odometry class for tracking robot pose
    private final SwerveDrivePoseEstimator m_poseEstimator;

    private double xVelocity = 0;
    private double yVelocity = 0;
    private double thetaVelociy = 0;

    private VisionDataProvider visionDataProvider;

    private boolean moving = false;

    // TODO: this looks unfinished?
    // public static final TrapezoidProfile.Constraints kXControllerConstraints =
    // new TrapezoidProfile.Constraints(kMaxSpeedMetersPerSecond,
    // kMaxAccelerationMetersPerSecondSquared);

    public final PIDController xController;
    public final PIDController yController;
    public ProfiledPIDController thetaController;

    public SwerveDriveSubsystem(double currentLimit) {
        final double Px = .15;
        final double Ix = 0;
        final double Dx = 0;
        final double xTolerance = 0.2;
        xController = new PIDController(Px, Ix, Dx);
        xController.setTolerance(xTolerance);

        final double Py = 0.15;
        final double Iy = 0;
        final double Dy = 0;
        final double yTolerance = 0.2;
        yController = new PIDController(Py, Iy, Dy);
        yController.setTolerance(yTolerance);

        final double Ptheta = 2;
        final double Itheta = 0;
        final double Dtheta = 0;
        final TrapezoidProfile.Constraints thetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
        thetaController = new ProfiledPIDController(Ptheta, Itheta, Dtheta, thetaControllerConstraints);

        switch (Identity.get()) {
            case SQUAREBOT:
                m_frontLeft = WCPModule(
                        "Front Left",
                        11, // drive CAN
                        30, // turn CAN
                        2, // turn encoder
                        0.812, // turn offset
                        currentLimit);
                m_frontRight = WCPModule(
                        "Front Right",
                        12, // drive CAN
                        32, // turn CAN
                        0, // turn encoder
                        0.382, // turn offset
                        currentLimit);
                m_rearLeft = WCPModule(
                        "Rear Left",
                        21, // drive CAN
                        31, // turn CAN
                        3, // turn encoder
                        0.172, // turn offset
                        currentLimit);
                m_rearRight = WCPModule(
                        "Rear Right",
                        22, // drive CAN
                        33, // turn CAN
                        1, // turn encoder
                        0.789, // turn offset
                        currentLimit);
                break;
            case SWERVE_TWO:
                m_frontLeft = AMModule(
                        "Front Left",
                        11, // drive CAN
                        0, // turn PWM
                        2, // turn encoder
                        0.012360, // turn offset
                        currentLimit);
                m_frontRight = AMModule(
                        "Front Right",
                        12, // drive CAN
                        2, // turn PWM
                        0, // turn encoder
                        0.543194, // turn offset
                        currentLimit);
                m_rearLeft = AMModule(
                        "Rear Left",
                        21, // drive CAN
                        1, // turn PWM
                        3, // turn encoder
                        0.744816, // turn offset
                        currentLimit);
                m_rearRight = AMModule(
                        "Rear Right",
                        22, // drive CAN
                        3, // turn PWM
                        1, // turn encoder
                        0.750468, // turn offset
                        currentLimit);
                break;
            case SWERVE_ONE: // TODO: verify these offsets (merge conflicts)
                m_frontLeft = AMModule(
                        "Front Left",
                        11, // drive CAN
                        0, // turn PWM
                        2, // turn encoder
                        0.984, // turn offset
                        currentLimit);
                m_frontRight = AMModule(
                        "Front Right",
                        12, // drive CAN
                        2, // turn PWM
                        0, // turn encoder
                        0.373, // turn offset
                        currentLimit);
                m_rearLeft = AMModule(
                        "Rear Left",
                        21, // drive CAN
                        1, // turn PWM
                        3, // turn encoder
                        0.719, // turn offset
                        currentLimit);
                m_rearRight = AMModule(
                        "Rear Right",
                        22, // drive CAN
                        3, // turn PWM
                        1, // turn encoder
                        0.688, // turn offset
                        currentLimit);
                break;
            default:
                throw new IllegalStateException("Identity is not swerve: " + Identity.get().name());
        }

        m_gyro = new AHRS(SerialPort.Port.kUSB);
        m_poseEstimator = new SwerveDrivePoseEstimator(
                kDriveKinematics,
                getHeading(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                new Pose2d(),
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.9, 0.9, Integer.MAX_VALUE));

        visionDataProvider = new VisionDataProvider(m_poseEstimator, () -> getMoving());
        SmartDashboard.putData("Drive Subsystem", this);
    }

    private static SwerveModule WCPModule(
            String name,
            int driveMotorCanId,
            int turningMotorCanId,
            int turningEncoderChannel,
            double turningOffset,
            double currentLimit) {
        final double kWheelDiameterMeters = 0.1005; // WCP 4 inch wheel
        final double kDriveReduction = 6.55; // see wcproducts.com
        final double driveEncoderDistancePerTurn = kWheelDiameterMeters * Math.PI / kDriveReduction;
        final double turningGearRatio = 1.0;
        final double kPModuleDriveController = 0.1;
        final double kPModuleTurningController = 0.5;
        final double kMaxModuleAngularSpeedRadiansPerSecond = 20 * Math.PI;
        final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 20 * Math.PI;

        FalconDriveMotor driveMotor = new FalconDriveMotor(name, driveMotorCanId, currentLimit);
        FalconDriveEncoder driveEncoder = new FalconDriveEncoder(name, driveMotor, driveEncoderDistancePerTurn);
        NeoTurningMotor turningMotor = new NeoTurningMotor(name, turningMotorCanId);
        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(name, turningEncoderChannel, turningOffset,
                turningGearRatio);
        PIDController driveController = new PIDController(kPModuleDriveController, 0, 0);
        ProfiledPIDController turningController = new ProfiledPIDController(
                kPModuleTurningController, 0, 0,
                new TrapezoidProfile.Constraints(
                        kMaxModuleAngularSpeedRadiansPerSecond,
                        kMaxModuleAngularAccelerationRadiansPerSecondSquared));
        turningController.enableContinuousInput(0, 2 * Math.PI);
        SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, .5);
        // TODO: real values for kS and kV.
        SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward(0.1, 0.005);

        return new SwerveModule(name, driveMotor, turningMotor, driveEncoder, turningEncoder,
                driveController, turningController, driveFeedforward, turningFeedforward);
    }

    private static SwerveModule AMModule(
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
        final double kPModuleDriveController = 0.1;
        final double kPModuleTurningController = 0.5;
        final double kMaxModuleAngularSpeedRadiansPerSecond = 20 * Math.PI;
        final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 20 * Math.PI;


        FalconDriveMotor driveMotor = new FalconDriveMotor(name, driveMotorCanId, currentLimit);
        FalconDriveEncoder driveEncoder = new FalconDriveEncoder(name, driveMotor, driveEncoderDistancePerTurn);
        PWMTurningMotor turningMotor = new PWMTurningMotor(name, turningMotorChannel);
        AnalogTurningEncoder turningEncoder = new AnalogTurningEncoder(name, turningEncoderChannel, turningOffset,
                turningGearRatio);
        PIDController driveController = new PIDController(kPModuleDriveController, 0, 0);
        ProfiledPIDController turningController = new ProfiledPIDController(
                kPModuleTurningController, 0, 0,
                new TrapezoidProfile.Constraints(
                        kMaxModuleAngularSpeedRadiansPerSecond,
                        kMaxModuleAngularAccelerationRadiansPerSecondSquared));
        turningController.enableContinuousInput(0, 2 * Math.PI);
        SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, .5);
        // TODO: real values for kS and kV.
        SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward(0.1, 0.005);

        return new SwerveModule(name, driveMotor, turningMotor, driveEncoder, turningEncoder,
                driveController, turningController, driveFeedforward, turningFeedforward);
    }

    public void updateOdometry() {
        m_poseEstimator.update(
                getHeading(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                });
        // {
        // if (m_pose.aprilPresent()) {
        // m_poseEstimator.addVisionMeasurement(
        // m_pose.getRobotPose(0),
        // Timer.getFPGATimestamp() - 0.3);
        // }
    }

    @Override
    public void periodic() {
        updateOdometry();
        RobotContainer.m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d robotPose) {
        m_poseEstimator.resetPosition(getHeading(), new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
        },
                robotPose);
    }

    public boolean getMoving() {
        return moving;
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (positive =
     *                      forward).
     * @param ySpeed        Speed of the robot in the y direction (positive =
     *                      leftward).
     * @param rot           Angular rate of the robot (positive = counterclockwise)
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // TODO Fix this number

        if (Math.abs(xSpeed) < .01)
            xSpeed = 100 * xSpeed * xSpeed * Math.signum(xSpeed);
        if (Math.abs(ySpeed) < .01)
            ySpeed = 100 * ySpeed * ySpeed * Math.signum(ySpeed);
        if (Math.abs(rot) < .1)
            rot = 0;
        var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(kMaxSpeedMetersPerSecond * xSpeed,
                                kMaxSpeedMetersPerSecond * ySpeed, kMaxAngularSpeedRadiansPerSecond * rot,
                                getPose().getRotation())
                        : new ChassisSpeeds(kMaxSpeedMetersPerSecond * xSpeed, kMaxSpeedMetersPerSecond * ySpeed,
                                kMaxAngularSpeedRadiansPerSecond * rot));

        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, kMaxSpeedMetersPerSecond);

        getRobotVelocity(swerveModuleStates);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);

        getRobotVelocity(desiredStates);

    }

    public void getRobotVelocity(SwerveModuleState[] desiredStates) {
        ChassisSpeeds chassisSpeeds = kDriveKinematics.toChassisSpeeds(desiredStates[0], desiredStates[1],
                desiredStates[2], desiredStates[3]);
        xVelocity = chassisSpeeds.vxMetersPerSecond;
        yVelocity = chassisSpeeds.vyMetersPerSecond;
        thetaVelociy = chassisSpeeds.omegaRadiansPerSecond;

        if (xVelocity >= 0.1 || yVelocity >= 0.1 || thetaVelociy >= 0.1) {
            moving = true;
        } else {
            moving = false;
        }

    }

    public void test(double[][] desiredOutputs) {
        m_frontLeft.setOutput(desiredOutputs[0][0], desiredOutputs[0][1]);
        m_frontRight.setOutput(desiredOutputs[1][0], desiredOutputs[1][1]);
        m_rearLeft.setOutput(desiredOutputs[2][0], desiredOutputs[2][1]);
        m_rearRight.setOutput(desiredOutputs[3][0], desiredOutputs[3][1]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetDriveEncoders();
        m_frontRight.resetDriveEncoders();
        m_rearLeft.resetDriveEncoders();
        m_rearRight.resetDriveEncoders();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-m_gyro.getFusedHeading());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("heading_degrees", () -> this.getHeading().getDegrees(), null);
        builder.addDoubleProperty("translationalx", () -> getPose().getX(), null);
        builder.addDoubleProperty("translationaly", () -> getPose().getY(), null);
        builder.addDoubleProperty("theta", () -> getPose().getRotation().getRadians(), null);
        builder.addDoubleProperty("Front Left Position", () -> m_frontLeft.getPosition().distanceMeters, null);
        builder.addDoubleProperty("Front Right Position", () -> m_frontRight.getPosition().distanceMeters, null);
        builder.addDoubleProperty("Rear Left Position", () -> m_rearLeft.getPosition().distanceMeters, null);
        builder.addDoubleProperty("Rear Right Position", () -> m_rearRight.getPosition().distanceMeters, null);
        builder.addDoubleProperty("Theta Controller Error", () -> thetaController.getPositionError(), null);
        builder.addDoubleProperty("Theta Controller Measurment", () -> getPose().getRotation().getRadians(), null);
        builder.addDoubleProperty("Theta Controller Setpoint", () -> thetaController.getSetpoint().position, null);

        builder.addDoubleProperty("Front Left Turning Controller Output", () -> m_frontLeft.getTControllerOutput(),
                null);
        builder.addDoubleProperty("Front Right Turning Controller Output", () -> m_frontRight.getTControllerOutput(),
                null);
        builder.addDoubleProperty("Rear Left Turning Controller Output", () -> m_rearLeft.getTControllerOutput(), null);
        builder.addDoubleProperty("Rear Right Turning Controller Output", () -> m_rearRight.getTControllerOutput(),
                null);
        builder.addDoubleProperty("Front Left Driving Controller Output", () -> m_frontLeft.getDControllerOutput(),
                null);
        builder.addDoubleProperty("Front Right Driving Controller Output", () -> m_frontRight.getDControllerOutput(),
                null);
        builder.addDoubleProperty("Rear Left Driving Controller Output", () -> m_rearLeft.getDControllerOutput(), null);
        builder.addDoubleProperty("Rear Right Driving Controller Output", () -> m_rearRight.getDControllerOutput(),
                null);

        builder.addDoubleProperty("X controller Error", () -> xController.getPositionError(), null);
        builder.addDoubleProperty("X controller Setpoint", () -> xController.getSetpoint(), null);
        builder.addDoubleProperty("X controller Measurment", () -> getPose().getX(), null);

        builder.addDoubleProperty("Y controller Error", () -> yController.getPositionError(), null);
        builder.addDoubleProperty("Y controller Setpoint", () -> yController.getSetpoint(), null);
        builder.addDoubleProperty("Y controller Measurment", () -> getPose().getY(), null);

        builder.addBooleanProperty("Moving", () -> getMoving(), null);

        builder.addDoubleProperty("X controller Velocity", () -> xVelocity, null);
        builder.addDoubleProperty("Y controller Velocity", () -> yVelocity, null);
        builder.addDoubleProperty("Theta controller Velocity", () -> thetaVelociy, null);

    }

}
