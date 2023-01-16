package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.localization.VisionDataProviderNT3;
import frc.robot.localization.VisionEstimate;
import frc.robot.subsystems.util.SwerveModuleFactory;
import edu.wpi.first.math.util.Units;

import frc.robot.Constants.KSwerve;

public class SwerveDrivetrain extends SubsystemBase {

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(KSwerve.kWheelBase / 2, KSwerve.kTrackWidth / 2),  // Front Left
            new Translation2d(KSwerve.kWheelBase / 2, -KSwerve.kTrackWidth / 2), // Front Right
            new Translation2d(-KSwerve.kWheelBase / 2, KSwerve.kTrackWidth / 2), // Rear Left
            new Translation2d(-KSwerve.kWheelBase / 2, -KSwerve.kTrackWidth / 2) // Rear Right
        );

    VisionDataProviderNT3 vision = new VisionDataProviderNT3();
    

    private final SwerveModule m_frontLeft = SwerveModuleFactory
            .newSwerveModuleWithFalconDriveAndAnalogSteeringEncoders(
                    "Front Left",
                    KSwerve.kDriveMotorIDs[0], // driveMotorChannel
                    KSwerve.kTurnMotorIDs[0], // turningMotorChannel
                    KSwerve.kTurnEncoderIDs[0], // turningEncoderChannel
                    KSwerve.kDriveEncoderReversed[0], // driveEncoderReversed
                    KSwerve.kTurnEncoderReversed[0], // steer encoder reverse
                    KSwerve.kTurningOffsets[0]);

    private final SwerveModule m_frontRight = SwerveModuleFactory
            .newSwerveModuleWithFalconDriveAndAnalogSteeringEncoders(
                    "Front Right",
                    KSwerve.kDriveMotorIDs[1], // driveMotorChannel
                    KSwerve.kTurnMotorIDs[1], // turningMotorChannel
                    KSwerve.kTurnEncoderIDs[1], // turningEncoderChannel
                    KSwerve.kDriveEncoderReversed[1], // driveEncoderReversed
                    KSwerve.kTurnEncoderReversed[1], // turningEncoderReversed
                    KSwerve.kTurningOffsets[1]);

    private final SwerveModule m_rearLeft = SwerveModuleFactory
            .newSwerveModuleWithFalconDriveAndAnalogSteeringEncoders(
                    "Rear Left",
                    KSwerve.kDriveMotorIDs[2], // driveMotorChannel
                    KSwerve.kTurnMotorIDs[2], // turningMotorChannel
                    KSwerve.kTurnEncoderIDs[2], // turningEncoderChannel
                    KSwerve.kDriveEncoderReversed[2], // driveEncoderReversed
                    KSwerve.kTurnEncoderReversed[2], // turningEncoderReversed
                    KSwerve.kTurningOffsets[2]);

    private final SwerveModule m_rearRight = SwerveModuleFactory
            .newSwerveModuleWithFalconDriveAndAnalogSteeringEncoders(
                    "Rear Right",
                    KSwerve.kDriveMotorIDs[3], // driveMotorChannel
                    KSwerve.kTurnMotorIDs[3], // turningMotorChannel
                    KSwerve.kTurnEncoderIDs[3], // turningEncoderChannel
                    KSwerve.kDriveEncoderReversed[3], // driveEncoderReversed
                    KSwerve.kTurnEncoderReversed[3], // turningEncoderReversed
                    KSwerve.kTurningOffsets[3]);

    // The gyro sensor. We have a Nav-X.
    private final NavX m_gyro;
    
    // Odometry class for tracking robot pose
    SwerveDrivePoseEstimator m_poseEstimator;

    public SwerveDrivetrain() {
        m_gyro = new NavX(new AHRS(SerialPort.Port.kUSB), false);
        m_poseEstimator = new SwerveDrivePoseEstimator(
            kDriveKinematics,
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_rearLeft.getPosition(),
              m_rearRight.getPosition()
            },
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(1)));
        SmartDashboard.putData("Drive Subsystem", this);

    }

    @Override
    public void periodic() {
        vision.periodic();

        m_poseEstimator.update(
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
        });

        VisionEstimate[] estimates = vision.getPoseEstimates();
        for(VisionEstimate estimate : estimates) {
            m_poseEstimator.addVisionMeasurement(
                estimate.getEstimatedPose(),
                estimate.getTimestamp());
        }
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
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(KSwerve.kMaxSpeedMetersPerSecond * xSpeed,
                                KSwerve.kMaxSpeedMetersPerSecond * ySpeed, KSwerve.kMaxAngularSpeedRadiansPerSecond * rot,
                                m_gyro.getRotation2d())
                        : new ChassisSpeeds(KSwerve.kMaxSpeedMetersPerSecond * xSpeed, KSwerve.kMaxSpeedMetersPerSecond * ySpeed,
                                KSwerve.kMaxAngularSpeedRadiansPerSecond * rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, KSwerve.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, KSwerve.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("heading_degrees", m_gyro::getHeading, null);
        builder.addDoubleProperty("translationalx", () -> m_poseEstimator.getEstimatedPosition().getX(), null);
        builder.addDoubleProperty("translationaly", () -> m_poseEstimator.getEstimatedPosition().getY(), null);
    }
}