package org.team100.lib.motion.drivetrain;

import java.io.FileWriter;

import org.team100.lib.motion.drivetrain.kinematics.FrameTransform;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase implements SwerveDriveSubsystemInterface {
    private final Heading m_heading;
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final Field2d m_field;
    private final FrameTransform m_frameTransform;

    private final SwerveLocal m_swerveLocal;

    // TODO: this looks broken
    public double keyList = -1;

    public SwerveDriveSubsystem(
            Heading heading,
            SwerveDrivePoseEstimator poseEstimator,
            FrameTransform frameTransform,
            SwerveLocal swerveLocal,
            Field2d field) {
        m_heading = heading;
        m_poseEstimator = poseEstimator;
        m_field = field;
        m_frameTransform = frameTransform;
        m_swerveLocal = swerveLocal;
    }

    public void updateOdometry() {
        m_poseEstimator.update(m_heading.getHeadingNWU(), m_swerveLocal.positions());
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
        m_poseEstimator.resetPosition(m_heading.getHeadingNWU(), m_swerveLocal.positions(), robotPose);
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
        ChassisSpeeds targetChassisSpeeds = m_frameTransform.fromFieldRelativeSpeeds(
                twist.dx, twist.dy, twist.dtheta, getPose().getRotation());
        m_swerveLocal.setChassisSpeeds(targetChassisSpeeds);
    }

    public void defense() {
        m_swerveLocal.defense();
    }

    public void test(double[][] desiredOutputs, FileWriter writer) {
        m_swerveLocal.test(desiredOutputs, writer);
    }

    @Override
    public void stop() {
        m_swerveLocal.stop();
    }

    ///////////////////////////////////////////////////////////

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

    // current pose in format that field2d can use
    private final NetworkTable field = inst.getTable("field");
    private final DoubleArrayPublisher robotPosePub = field.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = field.getStringTopic(".type").publish();
    {
        fieldTypePub.set("Field2d");
    }
}
