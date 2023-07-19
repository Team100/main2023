package org.team100.lib.motion.drivetrain;

import java.io.FileWriter;

import org.team100.lib.config.Identity;
import org.team100.lib.motion.drivetrain.kinematics.SwerveDriveKinematicsFactory;
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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase implements SwerveDriveSubsystemInterface {
    private final Heading m_heading;
    private final SpeedLimits m_speedLimits;
    private final SwerveDriveKinematics m_DriveKinematics;
    private final SwerveModuleCollection m_modules;
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final RedundantGyro m_gyro;
    private final Field2d m_field;
    private final VeeringCorrection m_veering;

    // TODO: this looks broken
    public double keyList = -1;

    public SwerveDriveSubsystem(
            Heading heading,
            SpeedLimits speedLimits,
            SwerveDriveKinematics driveKinematics,
            SwerveModuleCollection modules,
            SwerveDrivePoseEstimator poseEstimator,
            RedundantGyro gyro,
            Field2d field) {
        m_heading = heading;
        m_speedLimits = speedLimits;
        m_DriveKinematics = SwerveDriveKinematicsFactory.get(Identity.get());
        m_modules = modules;
        m_poseEstimator = poseEstimator;
        m_gyro = gyro;
        m_field = field;
        m_veering = new VeeringCorrection(m_gyro);
    }

    public void updateOdometry() {
        m_poseEstimator.update(m_heading.getHeading(), m_modules.positions());
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
     * @param twist Field coordinate velocities in meters and radians per second.
     */
    public void driveInFieldCoords(Twist2d twist) {
        Rotation2d robotAngle = m_veering.correct(getPose().getRotation());
        ChassisSpeeds targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                twist.dx, twist.dy, twist.dtheta, robotAngle);
        setChassisSpeeds(targetChassisSpeeds);
    }

    /**
     * Sets the wheels to make an "X" pattern
     * TODO: let the drivetrain decide to do this when it's stopped for more than
     * 0.25 s
     */
    public void defense() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = new SwerveModuleState(0, new Rotation2d(Math.PI / 4));
        states[1] = new SwerveModuleState(0, new Rotation2d(7 * Math.PI / 4));
        states[2] = new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4));
        states[3] = new SwerveModuleState(0, new Rotation2d(5 * Math.PI / 4));
        setModuleStates(states);
    }

    public void test(double[][] desiredOutputs, FileWriter writer) {
        m_modules.test(desiredOutputs, writer);
    }

    @Override
    public void stop() {
        m_modules.stop();
    }

    ///////////////////////////////////////////////////////////

    /** @param targetChassisSpeeds speeds in robot coordinates. */
    private void setChassisSpeeds(ChassisSpeeds targetChassisSpeeds) {
        desiredSpeedXPublisher.set(targetChassisSpeeds.vxMetersPerSecond);
        desiredSpeedYPublisher.set(targetChassisSpeeds.vyMetersPerSecond);
        desiredSpeedRotPublisher.set(targetChassisSpeeds.omegaRadiansPerSecond);
        SwerveModuleState[] targetModuleStates = m_DriveKinematics.toSwerveModuleStates(targetChassisSpeeds);
        setModuleStates(targetModuleStates);
    }

    private void setModuleStates(SwerveModuleState[] targetModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(targetModuleStates, m_speedLimits.kMaxSpeedMetersPerSecond);
        publishImpliedChassisSpeeds(targetModuleStates);
        m_modules.setDesiredStates(targetModuleStates);
    }

    /** publish chassis speeds implied by the module settings */
    private void publishImpliedChassisSpeeds(SwerveModuleState[] targetModuleStates) {
        ChassisSpeeds targetChassisSpeeds = m_DriveKinematics.toChassisSpeeds(
                targetModuleStates[0],
                targetModuleStates[1],
                targetModuleStates[2],
                targetModuleStates[3]);
        speedXPublisher.set(targetChassisSpeeds.vxMetersPerSecond);
        speedYPublisher.set(targetChassisSpeeds.vyMetersPerSecond);
        speedRotPublisher.set(targetChassisSpeeds.omegaRadiansPerSecond);
        movingPublisher.set(isMoving(targetChassisSpeeds));
    }

    private static boolean isMoving(ChassisSpeeds speeds) {
        return (speeds.vxMetersPerSecond >= 0.1
                || speeds.vyMetersPerSecond >= 0.1
                || speeds.omegaRadiansPerSecond >= 0.1);
    }

    // Do we need this?
    // private void resetEncoders() {
    // m_modules.resetEncoders();
    // }

    // observers
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
