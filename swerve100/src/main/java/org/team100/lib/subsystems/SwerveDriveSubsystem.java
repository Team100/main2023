package org.team100.lib.subsystems;

import java.io.FileWriter;
import java.io.IOException;

import org.team100.lib.config.Identity;
import org.team100.lib.sensors.RedundantGyro;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase implements SwerveDriveSubsystemInterface {

    private final Heading m_heading;
    private final SpeedLimits m_speedLimits;
    private final SwerveDriveKinematics m_DriveKinematics = SwerveDriveKinematicsFactory.get(Identity.get());
    private final SwerveModuleCollection m_modules;
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final RedundantGyro m_gyro;
    private final Field2d m_field;
    private final VeeringCorrection m_veering;

    private ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();
    private ChassisSpeeds actualChassisSpeeds = new ChassisSpeeds();

    // TODO: this looks broken
    public double keyList = -1;

    public SwerveDriveSubsystem(
            Heading heading,
            SpeedLimits speedLimits,
            SwerveDriveKinematics driveKinematics,
            SwerveModuleCollection modules,
            SwerveDrivePoseEstimator poseEstimator,
            RedundantGyro gyro,
            Field2d field)
            throws IOException {
        m_heading = heading;
        m_speedLimits = speedLimits;
        m_modules = modules;
        m_poseEstimator = poseEstimator;
        m_gyro = gyro;
        m_field = field;

        m_veering = new VeeringCorrection(m_gyro);
    }

    public void updateOdometry() {
        m_poseEstimator.update(
                m_heading.getHeading(),
                m_modules.positions());
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
        poseXPublisher.set(newEstimate.getX());
        poseYPublisher.set(newEstimate.getY());
        poseRotPublisher.set(newEstimate.getRotation().getRadians());
    }

    @Override
    public void periodic() {
        updateOdometry();
        m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
    }

    /**
     * Note this doesn't include the gyro reading directly, the estimate is
     * considerably massaged by the odometry logic.
     */
    @Override
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public double getRadians() {
        return m_poseEstimator.getEstimatedPosition().getRotation().getRadians();
    }

    public void resetPose(Pose2d robotPose) {
        m_poseEstimator.resetPosition(m_heading.getHeading(), m_modules.positions(), robotPose);
    }

    // TODO: this looks broken
    public void setKeyList() {
        keyList = NetworkTableInstance.getDefault().getTable("Vision").getKeys().size();
    }

    // TODO: this looks broken
    public double getVisionSize() {
        return keyList;
    }

    /**
     * Calibrated inputs
     * 
     * TODO: replace this logic with the ChassisSpeedFactory logic
     * 
     * @param twist meters and radians per second
     */
    @Override
    public void driveMetersPerSec(Twist2d twist, boolean fieldRelative) {
        if (fieldRelative) {
            driveInFieldCoords(twist.dx, twist.dy, twist.dtheta);
        } else {
            driveInRobotCoords(twist.dx, twist.dy, twist.dtheta);
        }
    }

    /** Field coordinates in meters and radians per second.  */
    public void driveInFieldCoords(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond) {
        Rotation2d robotAngle = m_veering.correct(getPose().getRotation());
        ChassisSpeeds targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                vxMetersPerSecond,
                vyMetersPerSecond,
                omegaRadiansPerSecond,
                robotAngle);
        setChassisSpeeds(targetChassisSpeeds);
    }

    /** Robot coordinates in meters and radians per second. */
    public void driveInRobotCoords(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond) {
        ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds(
                vxMetersPerSecond,
                vyMetersPerSecond,
                omegaRadiansPerSecond);
        setChassisSpeeds(targetChassisSpeeds);
    }

    /** @param targetChassisSpeeds speeds in robot coordinates. */
    public void setChassisSpeeds(ChassisSpeeds targetChassisSpeeds) {
        desiredChassisSpeeds = targetChassisSpeeds;

        SwerveModuleState[] swerveModuleStates = m_DriveKinematics.toSwerveModuleStates(targetChassisSpeeds);
        setModuleStates(swerveModuleStates);

        speedXPublisher.set(desiredChassisSpeeds.vxMetersPerSecond);
        speedYPublisher.set(desiredChassisSpeeds.vyMetersPerSecond);
        speedRotPublisher.set(desiredChassisSpeeds.omegaRadiansPerSecond);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, m_speedLimits.kMaxSpeedMetersPerSecond);
        getRobotVelocity(desiredStates);
        m_modules.setDesiredStates(desiredStates);
    }

    public void getRobotVelocity(SwerveModuleState[] desiredStates) {
        actualChassisSpeeds = m_DriveKinematics.toChassisSpeeds(desiredStates[0], desiredStates[1],
                desiredStates[2], desiredStates[3]);
        speedXPublisher.set(actualChassisSpeeds.vxMetersPerSecond);
        speedYPublisher.set(actualChassisSpeeds.vyMetersPerSecond);
        speedRotPublisher.set(actualChassisSpeeds.omegaRadiansPerSecond);
        movingPublisher.set(isMoving());
    }

    private boolean isMoving() {
        return (actualChassisSpeeds.vxMetersPerSecond >= 0.1
                || actualChassisSpeeds.vyMetersPerSecond >= 0.1
                || actualChassisSpeeds.omegaRadiansPerSecond >= 0.1);
    }

    public void test(double[][] desiredOutputs, FileWriter writer) {
        m_modules.test(desiredOutputs);
        try {
            if (writer != null) {
                writer.write("Timestamp: " + Timer.getFPGATimestamp() +
                        ", P" + m_modules.positions()[0].distanceMeters
                        + ", " + m_modules.states()[0].speedMetersPerSecond + "\n");
                writer.flush();
            }
        } catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_modules.resetEncoders();
    }

    @Override
    public void stop() {
        m_modules.stop();
    }

    // for observers

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // current pose
    private final NetworkTable pose = inst.getTable("current pose");
    private final DoublePublisher poseXPublisher = pose.getDoubleTopic("x").publish();
    private final DoublePublisher poseYPublisher = pose.getDoubleTopic("y").publish();
    private final DoublePublisher poseRotPublisher = pose.getDoubleTopic("theta").publish();

    // desired speed
    private final NetworkTable desired = inst.getTable("desired speed");
    private final DoublePublisher desiredSpeedXPublisher = desired.getDoubleTopic("x").publish();
    private final DoublePublisher desiredSpeedYPublisher = desired.getDoubleTopic("y").publish();
    private final DoublePublisher desiredSpeedRotPublisher = desired.getDoubleTopic("theta").publish();

    // actual speed
    private final NetworkTable speed = inst.getTable("actual speed");
    private final DoublePublisher speedXPublisher = speed.getDoubleTopic("x").publish();
    private final DoublePublisher speedYPublisher = speed.getDoubleTopic("y").publish();
    private final DoublePublisher speedRotPublisher = speed.getDoubleTopic("theta").publish();
    private final BooleanPublisher movingPublisher = speed.getBooleanTopic("moving").publish();

    // current pose in format that field2d can use
    private final NetworkTable field = inst.getTable("field");
    private final DoubleArrayPublisher robotPosePub = field.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = field.getStringTopic(".type").publish();
    {
        fieldTypePub.set("Field2d");
    }
}
