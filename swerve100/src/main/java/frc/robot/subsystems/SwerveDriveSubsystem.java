package frc.robot.subsystems;

import java.io.IOException;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import team100.config.Identity;
import team100.localization.VisionDataProvider;

public class SwerveDriveSubsystem extends SubsystemBase {
    // TODO: make this an instance var
    public static final SwerveDriveKinematics kDriveKinematics;
    // public ChassisSpeeds robotStates = new ChassisSpeeds();
    // public double observedVelocity;
    public ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();

    static {
        final double kTrackWidth;
        final double kWheelBase;
        switch (Identity.get()) {
            case COMP_BOT:
                kTrackWidth = 0.491;
                kWheelBase = 0.765;
                break;
            case SWERVE_TWO:
                kTrackWidth = 0.380;
                kWheelBase = 0.445;
                break;
            case SWERVE_ONE:
                kTrackWidth = 0.449;
                kWheelBase = 0.464;
                break;
            case FROM_8048:
                kTrackWidth = 0.46;
                kWheelBase = 0.55; // approximate
                break;
            case BLANK: // for simulation
                kTrackWidth = 0.5;
                kWheelBase = 0.5;
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

    public final double kMaxSpeedMetersPerSecond;
    public final double kMaxAccelerationMetersPerSecondSquared;
    public final double kMaxAngularSpeedRadiansPerSecond;
    public final double kMaxAngularSpeedRadiansPerSecondSquared;

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

    private double x;
    private double y;
    private double rotation;
    private boolean isFieldRelative;

    public VisionDataProvider visionDataProvider;

    private boolean moving = false;

    public final PIDController xController;
    public final PIDController yController;
    public final ProfiledPIDController headingController;
    public ProfiledPIDController thetaController;

    private final DoubleArrayPublisher robotPosePub;
    private final StringPublisher fieldTypePub;

    public SwerveDriveSubsystem(DriverStation.Alliance alliance, double currentLimit) throws IOException {
        // Sets up Field2d pose tracking for glass.
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable fieldTable = inst.getTable("field");
        robotPosePub = fieldTable.getDoubleArrayTopic("robotPose").publish();
        fieldTypePub = fieldTable.getStringTopic(".type").publish();
        fieldTypePub.set("Field2d");

        switch (Identity.get()) {
            case COMP_BOT:
                headingController = new ProfiledPIDController( //
                        1, // kP
                        .5, // kI
                        0.15, // kD
                        new TrapezoidProfile.Constraints(
                                2 * Math.PI, // speed rad/s
                                2 * Math.PI)); // accel rad/s/s
                headingController.setIntegratorRange(-0.1, 0.1);
                // Note very low heading tolerance.
                headingController.setTolerance(Units.degreesToRadians(0.1));
                kMaxSpeedMetersPerSecond = 5;
                kMaxAccelerationMetersPerSecondSquared = 10;
                kMaxAngularSpeedRadiansPerSecond = 5;
                kMaxAngularSpeedRadiansPerSecondSquared = 5;
                xController = new PIDController(
                        0.15, // kP
                        0.0, // kI
                        0.0); // kD
                xController.setTolerance(0.2);

                yController = new PIDController(
                        0.15, // kP
                        0.0, // kI
                        0.0); // kD
                yController.setTolerance(0.2);

                thetaController = new ProfiledPIDController(
                        3.0, // kP
                        0.0, // kI
                        0.0, // kD
                        new TrapezoidProfile.Constraints(
                                kMaxAngularSpeedRadiansPerSecond,
                                kMaxAngularSpeedRadiansPerSecondSquared));
                m_frontLeft = SwerveModuleFactory.WCPModule(
                        "Front Left",   
                        11, // drive CAN
                        30, // turn CAN
                        0, // turn encoder
                        0.26, // turn offset
                        currentLimit);
                m_frontRight = SwerveModuleFactory.WCPModule(
                        "Front Right",
                        12, // drive CAN
                        32, // turn CAN
                        1, // turn encoder
                        0.87, // turn offset
                        currentLimit);
                m_rearLeft = SwerveModuleFactory.WCPModule(
                        "Rear Left",
                        21, // drive CAN
                        31, // turn CAN
                        2, // turn encoder
                        0.27, // turn offset
                        currentLimit);
                m_rearRight = SwerveModuleFactory.WCPModule(
                        "Rear Right",
                        22, // drive CAN
                        33, // turn CAN
                        3, // turn encoder
                        0.46, // turn offset
                        currentLimit);
                break;
            case SWERVE_TWO:
                headingController = new ProfiledPIDController( //
                        1, // kP
                        .5, // kI
                        0.15, // kD
                        new TrapezoidProfile.Constraints(
                                2 * Math.PI, // speed rad/s
                                2 * Math.PI)); // accel rad/s/s

                headingController.setIntegratorRange(-0.1, 0.1);
                // Note very low heading tolerance.
                headingController.setTolerance(Units.degreesToRadians(0.1));
                kMaxSpeedMetersPerSecond = 5;
                kMaxAccelerationMetersPerSecondSquared = 10;
                kMaxAngularSpeedRadiansPerSecond = 5;
                kMaxAngularSpeedRadiansPerSecondSquared = 5;
                
                xController = new PIDController(
                        0.15, // kP
                        0.0, // kI
                        0.0); // kD
                xController.setTolerance(0.2);

                yController = new PIDController(
                        0.15, // kP
                        0.0, // kI
                        0.0); // kD
                yController.setTolerance(0.2);

                thetaController = new ProfiledPIDController(
                        3.0, // kP
                        0.0, // kI
                        0.0, // kD
                        new TrapezoidProfile.Constraints(
                                kMaxAngularSpeedRadiansPerSecond,
                                kMaxAngularSpeedRadiansPerSecondSquared));
                m_frontLeft = SwerveModuleFactory.AMModule(
                        "Front Left",
                        11, // drive CAN
                        3, // turn PWM
                        1, // turn encoder
                        0.032635, // turn offset
                        currentLimit);
                m_frontRight = SwerveModuleFactory.AMModule(
                        "Front Right",
                        12, // drive CAN
                        1, // turn PWM
                        3, // turn encoder
                        0.083566, // turn offset
                        currentLimit);
                m_rearLeft = SwerveModuleFactory.AMModule(
                        "Rear Left",
                        21, // drive CAN
                        2, // turn PWM
                        0, // turn encoder
                        0.747865, // turn offset
                        currentLimit);
                m_rearRight = SwerveModuleFactory.AMModule(
                        "Rear Right",
                        22, // drive CAN
                        0, // turn PWM
                        2, // turn encoder
                        0.894767, // turn offset
                        currentLimit);
                break;
            case SWERVE_ONE:
                headingController = new ProfiledPIDController( //
                        1, // kP
                        .5, // kI
                        0.15, // kD
                        new TrapezoidProfile.Constraints(
                                2 * Math.PI, // speed rad/s
                                2 * Math.PI)); // accel rad/s/s

                headingController.setIntegratorRange(-0.1, 0.1);
                // Note very low heading tolerance.
                headingController.setTolerance(Units.degreesToRadians(0.1));
                kMaxSpeedMetersPerSecond = 5;
                kMaxAccelerationMetersPerSecondSquared = 10;
                kMaxAngularSpeedRadiansPerSecond = 5;
                kMaxAngularSpeedRadiansPerSecondSquared = 5;
                xController = new PIDController(
                        0.15, // kP
                        0.0, // kI
                        0.0); // kD
                xController.setTolerance(0.2);

                yController = new PIDController(
                        0.15, // kP
                        0.0, // kI
                        0.0); // kD
                yController.setTolerance(0.2);

                thetaController = new ProfiledPIDController(
                        3.0, // kP
                        0.0, // kI
                        0.0, // kD
                        new TrapezoidProfile.Constraints(
                                kMaxAngularSpeedRadiansPerSecond,
                                kMaxAngularSpeedRadiansPerSecondSquared));
                m_frontLeft = SwerveModuleFactory.AMModule(
                        "Front Left",
                        11, // drive CAN
                        0, // turn PWM0
                        3, // turn encoder
                        0.69, // turn offset
                        currentLimit);
                m_frontRight = SwerveModuleFactory.AMModule(
                        "Front Right",
                        12, // drive CAN
                        2, // turn PWM
                        0, // turn encoder
                        0.72, // turn offset
                        currentLimit);
                m_rearLeft = SwerveModuleFactory.AMModule(
                        "Rear Left",
                        21, // drive CAN
                        1, // turn PWM
                        2, // turn encoder
                        0.37, // turn offset
                        currentLimit);
                m_rearRight = SwerveModuleFactory.AMModule(
                        "Rear Right",
                        22, // drive CAN
                        3, // turn PWM
                        1, // turn encoder
                        0.976726, // turn offset
                        currentLimit);
                break;
            case FROM_8048:
                headingController = new ProfiledPIDController( //
                        1, // kP
                        .5, // kI
                        0.15, // kD
                        new TrapezoidProfile.Constraints(
                                2 * Math.PI, // speed rad/s
                                2 * Math.PI)); // accel rad/s/s

                headingController.setIntegratorRange(-0.1, 0.1);
                // Note very low heading tolerance.
                headingController.setTolerance(Units.degreesToRadians(0.1));
                kMaxSpeedMetersPerSecond = 5;
                kMaxAccelerationMetersPerSecondSquared = 10;
                kMaxAngularSpeedRadiansPerSecond = 5;
                kMaxAngularSpeedRadiansPerSecondSquared = 5;
                xController = new PIDController(
                        0.15, // kP
                        0.0, // kI
                        0.0); // kD
                xController.setTolerance(0.2);

                yController = new PIDController(
                        0.15, // kP
                        0.0, // kI
                        0.0); // kD
                yController.setTolerance(0.2);

                thetaController = new ProfiledPIDController(
                        3.0, // kP
                        0.0, // kI
                        0.0, // kD
                        new TrapezoidProfile.Constraints(
                                kMaxAngularSpeedRadiansPerSecond,
                                kMaxAngularSpeedRadiansPerSecondSquared));
                m_frontLeft = SwerveModuleFactory.AMCANModule(
                        "Front Left",
                        3, // drive CAN
                        8, // turn CAN
                        1, // turn encoder (confirmed)
                        0.355157, // turn offset
                        currentLimit);
                m_frontRight = SwerveModuleFactory.AMCANModule(
                        "Front Right",
                        2, // drive CAN
                        6, // turn CAN
                        0, // turn encoder (confirmed)
                        0.404786, // turn offset
                        currentLimit);
                m_rearLeft = SwerveModuleFactory.AMCANModule(
                        "Rear Left",
                        1, // drive CAN
                        9, // turn CAN
                        3, // turn encoder (confirmed)
                        0.238757, // turn offset
                        currentLimit);
                m_rearRight = SwerveModuleFactory.AMCANModule(
                        "Rear Right",
                        4, // drive CAN
                        7, // turn CAN
                        2, // turn encoder (confirmed)
                        0.233683, // turn offset
                        currentLimit);
                break;
            case BLANK: // for simulation; just like squarebot for now
                headingController = new ProfiledPIDController( //
                        1, // kP
                        .5, // kI
                        0.15, // kD
                        new TrapezoidProfile.Constraints(
                                2 * Math.PI, // speed rad/s
                                2 * Math.PI)); // accel rad/s/s

                headingController.setIntegratorRange(-0.1, 0.1);
                // Note very low heading tolerance.
                headingController.setTolerance(Units.degreesToRadians(0.1));

                kMaxSpeedMetersPerSecond = 5;
                kMaxAccelerationMetersPerSecondSquared = 10;
                kMaxAngularSpeedRadiansPerSecond = 5;
                kMaxAngularSpeedRadiansPerSecondSquared = 5;
                xController = new PIDController(
                        0.15, // kP
                        0.0, // kI
                        0.0); // kD
                xController.setTolerance(0.2);

                yController = new PIDController(
                        0.15, // kP
                        0.0, // kI
                        0.0); // kD
                yController.setTolerance(0.2);

                thetaController = new ProfiledPIDController(
                        3.0, // kP
                        0.0, // kI
                        0.0, // kD
                        new TrapezoidProfile.Constraints(
                                kMaxAngularSpeedRadiansPerSecond,
                                kMaxAngularSpeedRadiansPerSecondSquared));
                m_frontLeft = SwerveModuleFactory.WCPModule(
                        "Front Left",
                        11, // drive CAN
                        30, // turn CAN
                        2, // turn encoder
                        0.812, // turn offset
                        currentLimit);
                m_frontRight = SwerveModuleFactory.WCPModule(
                        "Front Right",
                        12, // drive CAN
                        32, // turn CAN
                        0, // turn encoder
                        0.382, // turn offset
                        currentLimit);
                m_rearLeft = SwerveModuleFactory.WCPModule(
                        "Rear Left",
                        21, // drive CAN
                        31, // turn CAN
                        3, // turn encoder
                        0.172, // turn offset
                        currentLimit);
                m_rearRight = SwerveModuleFactory.WCPModule(
                        "Rear Right",
                        22, // drive CAN
                        33, // turn CAN
                        1, // turn encoder
                        0.789, // turn offset
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
                VecBuilder.fill(0.03, 0.03, 0.03),
                VecBuilder.fill(0.01, 0.01, Integer.MAX_VALUE));
        visionDataProvider = new VisionDataProvider(alliance, m_poseEstimator, () -> getPose());

        SmartDashboard.putData("Drive Subsystem", this);
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

        // Update the Field2d widget
        Pose2d newEstimate = m_poseEstimator.getEstimatedPosition();
        robotPosePub.set(new double[] {
                newEstimate.getX(),
                newEstimate.getY(),
                newEstimate.getRotation().getDegrees()
        });
    }

    @Override
    public void periodic() {
        updateOdometry();
        RobotContainer.m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
    }

    /**
     * Note this doesn't include the gyro reading directly, the estimate is
     * considerably massaged by the odometry logic.
     */
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public double getRadians() {
        return m_poseEstimator.getEstimatedPosition().getRotation().getRadians();
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
     *                      forward). X is between -1 and 1
     * @param ySpeed        Speed of the robot in the y direction (positive =
     *                      leftward). Y is between -1 and 1
     * @param rot           Angular rate of the robot (positive = counterclockwise)
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        xSpeed = MathUtil.clamp(xSpeed, -1, 1);
        ySpeed = MathUtil.clamp(ySpeed, -1, 1);
        rot = MathUtil.clamp(rot, -1, 1);
        // if (Math.abs(xSpeed) < .01)
        // xSpeed = 100 * xSpeed * xSpeed * Math.signum(xSpeed);
        // if (Math.abs(ySpeed) < .01)
        // ySpeed = 100 * ySpeed * ySpeed * Math.signum(ySpeed);
        if (Math.abs(rot) < .01)
            rot = 0;
        double gyroRate = m_gyro.getRate() * 0.25;
        Rotation2d rotation2 = getPose().getRotation().minus(new Rotation2d(gyroRate));
        desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(kMaxSpeedMetersPerSecond * xSpeed,
                kMaxSpeedMetersPerSecond * ySpeed, kMaxAngularSpeedRadiansPerSecond * rot,
                rotation2);
        // TODO fix fieldRelative making this go crazy when it is off
        var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? desiredChassisSpeeds
                        : new ChassisSpeeds(kMaxSpeedMetersPerSecond * xSpeed, kMaxSpeedMetersPerSecond * ySpeed,
                                kMaxAngularSpeedRadiansPerSecond * rot));

        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, kMaxSpeedMetersPerSecond);

        getRobotVelocity(swerveModuleStates);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        System.out.println(desiredChassisSpeeds);
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

    public ChassisSpeeds getRobotStates() {
        ChassisSpeeds chassisSpeeds = kDriveKinematics.toChassisSpeeds(
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState());
        return chassisSpeeds;
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
        return Rotation2d.fromDegrees(-m_gyro.getYaw());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("heading_radians", () -> 2 + this.getHeading().getRadians(), null);
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
        builder.addDoubleProperty("Pitch", () -> m_gyro.getPitch(), null);
        builder.addDoubleProperty("Roll", () -> m_gyro.getRoll(), null);
        builder.addDoubleProperty("Heading Degrees", () -> getHeading().getDegrees(), null);
        builder.addDoubleProperty("Heading Radians", () -> getHeading().getRadians(), null);
        builder.addDoubleProperty("Compass Heading", () -> m_gyro.getCompassHeading(), null);
        builder.addDoubleProperty("Angle", () -> m_gyro.getAngle(), null);
        builder.addDoubleProperty("xSpeed", () -> x, null);
        builder.addDoubleProperty("ySpeed", () -> y, null);
        builder.addDoubleProperty("Rotation", () -> rotation, null);
        builder.addBooleanProperty("Field Relative", () -> isFieldRelative, null);
        // builder.addDoubleProperty("Identity", () -> Identity.get(), null);

        builder.addDoubleProperty("Front Left Output", () -> m_frontLeft.getDriveOutput(), null);
        builder.addDoubleProperty("Front Right Output", () -> m_frontRight.getDriveOutput(), null);
        builder.addDoubleProperty("Rear Left Output", () -> m_rearLeft.getDriveOutput(), null);
        builder.addDoubleProperty("Rear Right Output", () -> m_rearLeft.getDriveOutput(), null);

        // builder.addDoubleProperty("Speed Ms Odometry", () -> observedVelocity, null);
        builder.addDoubleProperty("ChassisSpeedDesired Odometry X", () -> desiredChassisSpeeds.vxMetersPerSecond, null);
        builder.addDoubleProperty("ChassisSpeedDesired Odometry Y", () -> desiredChassisSpeeds.vyMetersPerSecond, null);
        builder.addDoubleProperty("GYRO ROLL", () -> m_gyro.getRoll(), null);

        builder.addDoubleProperty("GYRO Fused", () -> m_gyro.getFusedHeading(), null);
        builder.addDoubleProperty("Gyro Rate", () -> m_gyro.getRate(), null);
        builder.addDoubleProperty("getAngle", () -> m_gyro.getAngle() % 360, null);

    }
}
