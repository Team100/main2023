package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.RobotContainer;
import frc.robot.localization.VisionDataProvider;
import team100.config.Identity;

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

    public static final double ksVolts = 2;
    public static final double kvVoltSecondsPerMeter = 2.0;
    public static final double kaVoltSecondsSquaredPerMeter = 0.5;

    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = -5;

    private final SwerveModule m_frontLeft ;
    private final SwerveModule m_frontRight ;
    private final SwerveModule m_rearLeft ;
    private final SwerveModule m_rearRight ;

    // The gyro sensor. We have a Nav-X.
    public final AHRS m_gyro;
    // Odometry class for tracking robot pose
    SwerveDrivePoseEstimator m_poseEstimator;

    double xVelocity = 0;
    double yVelocity = 0;
    double thetaVelociy = 0;

    VisionDataProvider visionDataProvider;

    boolean moving = false;

    public PIDController xController = new PIDController(AutoConstants.kPXController, AutoConstants.kIXController,
            AutoConstants.kDXController);

    public PIDController yController = new PIDController(AutoConstants.kPYController, AutoConstants.kIYController,
            AutoConstants.kDYController);
    public ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController,
            AutoConstants.kThetaControllerConstraints);

    public SwerveDriveSubsystem() {
        switch (Identity.get()) {
            case SQUAREBOT:
                m_frontLeft = SwerveModuleFactory.frontLeft(0.812);
                m_frontRight = SwerveModuleFactory.frontRight(0.382);
                m_rearLeft = SwerveModuleFactory.rearLeft(0.172);
                m_rearRight = SwerveModuleFactory.rearRight(0.789);
                break;
            case SWERVE_TWO:
                m_frontLeft = SwerveModuleFactory.frontLeft(0.012360);
                m_frontRight = SwerveModuleFactory.frontRight(0.543194);
                m_rearLeft = SwerveModuleFactory.rearLeft(0.744816);
                m_rearRight = SwerveModuleFactory.rearRight(0.750468);
                break;
            case SWERVE_ONE: // TODO: verify these offsets (merge conflicts)
                m_frontLeft = SwerveModuleFactory.frontLeft(0.984);
                m_frontRight = SwerveModuleFactory.frontRight(0.373);
                m_rearLeft = SwerveModuleFactory.rearLeft(0.719);
                m_rearRight = SwerveModuleFactory.rearRight(0.688);
                break;
            default:
                throw new IllegalStateException("Identity is not swerve: " + Identity.get().name());
        }

        xController.setTolerance(0.2);
        yController.setTolerance(0.2);

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
                VecBuilder.fill(0.9, 0.9, Integer.MAX_VALUE)
        );

        visionDataProvider = new VisionDataProvider(m_poseEstimator, ()-> getMoving());
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

    public boolean getMoving(){
        return moving;
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        //TODO Fix this number

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


        if(xVelocity >= 0.1 || yVelocity >= 0.1 || thetaVelociy >= 0.1){
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

        builder.addDoubleProperty("Front Left Turning Controller Output", () -> m_frontLeft.getTControllerOutput(), null);
        builder.addDoubleProperty("Front Right Turning Controller Output", () -> m_frontRight.getTControllerOutput(), null);
        builder.addDoubleProperty("Rear Left Turning Controller Output", () -> m_rearLeft.getTControllerOutput(), null);
        builder.addDoubleProperty("Rear Right Turning Controller Output", () -> m_rearRight.getTControllerOutput(), null);
        builder.addDoubleProperty("Front Left Driving Controller Output", () -> m_frontLeft.getDControllerOutput(), null);
        builder.addDoubleProperty("Front Right Driving Controller Output", () -> m_frontRight.getDControllerOutput(), null);
        builder.addDoubleProperty("Rear Left Driving Controller Output", () -> m_rearLeft.getDControllerOutput(), null);
        builder.addDoubleProperty("Rear Right Driving Controller Output", () -> m_rearRight.getDControllerOutput(), null);

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
