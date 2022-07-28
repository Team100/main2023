package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This is a copy of DriveSubsystem for the second AndyMark swerve base.
 * 
 * It would be good to combine this with the DriveSubsystem somehow.
 */
public class Swerve2DriveSubsystem extends SubsystemBase {

    public static final double kTrackWidth = 0.38;
    public static final double kWheelBase = 0.445;

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    public static final double ksVolts = 2;
    public static final double kvVoltSecondsPerMeter = 2.0;
    public static final double kaVoltSecondsSquaredPerMeter = 0.5;

    public static final double kMaxSpeedMetersPerSecond = 5.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = 20;
    public static double m_northOffset = 0;

    private final SwerveModule m_frontLeft = SwerveModuleFactory
            .newSwerveModuleWithFalconDriveAndAnalogSteeringEncoders(
                    "Front Left",
                    11,
                    3, // motor
                    1, // encoder
                    false, // drive reverse
                    false, // steer encoder reverse
                    .185);

    private final SwerveModule m_frontRight = SwerveModuleFactory
            .newSwerveModuleWithFalconDriveAndAnalogSteeringEncoders(
                    "Front Right",
                    12,
                    1, // motor
                    3, // encoder
                    false, // drive reverse
                    false, // steer encoder reverse
                    .8934);

    private final SwerveModule m_rearLeft = SwerveModuleFactory
            .newSwerveModuleWithFalconDriveAndAnalogSteeringEncoders(
                    "Rear Left",
                    21,
                    2, // motor
                    0, // encoder
                    false, // drive reverse
                    false, // steer encoder reverse
                    0.7421);

    private final SwerveModule m_rearRight = SwerveModuleFactory
            .newSwerveModuleWithFalconDriveAndAnalogSteeringEncoders(
                    "Rear Right",
                    22,
                    0, // motor
                    2, // encoder
                    false, // drive reverse
                    false, // steer encoder reverse
                    .7466);

    // The gyro sensor. We have a Nav-X.
    private final AHRS m_gyro;
    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry;

    public Swerve2DriveSubsystem() {
        m_gyro = new AHRS(SerialPort.Port.kUSB);
        m_odometry = new SwerveDriveOdometry(kDriveKinematics, Rotation2d.fromDegrees(-m_gyro.getFusedHeading()+m_northOffset));
        SmartDashboard.putData("Drive Subsystem", this);
    }

    @Override
    public void periodic() {
        m_odometry.update(
                Rotation2d.fromDegrees(-m_gyro.getFusedHeading()+m_northOffset),
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState());
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(-m_gyro.getFusedHeading()+m_northOffset));
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
                                Rotation2d.fromDegrees(-m_gyro.getFusedHeading()+m_northOffset))
                        : new ChassisSpeeds(kMaxSpeedMetersPerSecond * xSpeed, kMaxSpeedMetersPerSecond * ySpeed,
                                kMaxAngularSpeedRadiansPerSecond * rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, kMaxSpeedMetersPerSecond);
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
    }

    public void test(double[][] desiredOutputs) {
        System.out.println("set outputs");
        m_frontLeft.setOutput(desiredOutputs[0][0], desiredOutputs[0][1]);
        m_frontRight.setOutput(desiredOutputs[1][0], desiredOutputs[1][1]);
        m_rearLeft.setOutput(desiredOutputs[2][0], desiredOutputs[2][1]);
        m_rearRight.setOutput(desiredOutputs[3][0], desiredOutputs[3][1]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
    }

    public double getHeading() {
        return Rotation2d.fromDegrees(-m_gyro.getFusedHeading()+m_northOffset).getDegrees();
    }

    public double getTurnRate() {
        return m_gyro.getRate() * (kGyroReversed ? -1.0 : 1.0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("heading_degrees", this::getHeading, null);
        builder.addDoubleProperty("translationalx", () -> getPose().getX(), null);
        builder.addDoubleProperty("translationaly", () -> getPose().getY(), null);
    }

    public void resetAHRS2() {
        m_northOffset = Rotation2d.fromDegrees(m_gyro.getFusedHeading()).getDegrees();
    }
}