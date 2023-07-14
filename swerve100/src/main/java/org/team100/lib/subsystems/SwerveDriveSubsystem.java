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
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase {
    public static final SwerveDriveKinematics kDriveKinematics = SwerveDriveKinematicsFactory.get(Identity.get());

    private final Heading m_heading;
    private final RedundantGyro m_gyro;
    private final Field2d m_field;
    private final SpeedLimits m_speedLimits;
    private final SwerveModuleCollection m_modules;
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final VeeringCorrection m_veering;

    // for observers
    private final DoubleArrayPublisher robotPosePub;
    private final StringPublisher fieldTypePub;
    private ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();
    private ChassisSpeeds actualChassisSpeeds= new ChassisSpeeds();

    // TODO: this looks broken
    public double keyList = -1;

    public SwerveDriveSubsystem(
            Heading heading,
            SpeedLimits speedLimits,
            SwerveModuleCollection modules,
            SwerveDrivePoseEstimator poseEstimator,
            double currentLimit,
            RedundantGyro gyro,
            Field2d field)
            throws IOException {
        m_heading = heading;
        m_gyro = gyro;
        m_field = field;
        m_veering = new VeeringCorrection(m_gyro);

        // Sets up Field2d pose tracking for glass.
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable fieldTable = inst.getTable("field");
        robotPosePub = fieldTable.getDoubleArrayTopic("robotPose").publish();
        fieldTypePub = fieldTable.getStringTopic(".type").publish();
        fieldTypePub.set("Field2d");

        m_speedLimits = speedLimits;
        m_modules = modules;
        m_poseEstimator = poseEstimator;

        SmartDashboard.putData("Drive Subsystem", this);
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
    public void driveMetersPerSec(Twist2d twist, boolean fieldRelative) {
        Rotation2d rotation2 = m_veering.correct(getPose().getRotation());
        desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(twist.dx, twist.dy, twist.dtheta, rotation2);
        ChassisSpeeds targetChassisSpeeds = fieldRelative ? desiredChassisSpeeds
                : new ChassisSpeeds(twist.dx, twist.dy, twist.dtheta);
        SwerveModuleState[] swerveModuleStates = kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds);
        setModuleStates(swerveModuleStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, m_speedLimits.kMaxSpeedMetersPerSecond);
        getRobotVelocity(desiredStates);
        m_modules.setDesiredStates(desiredStates);
    }

    public void getRobotVelocity(SwerveModuleState[] desiredStates) {
        actualChassisSpeeds = kDriveKinematics.toChassisSpeeds(desiredStates[0], desiredStates[1],
                desiredStates[2], desiredStates[3]);
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

    public void stop() {
        m_modules.stop();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Pose X", () -> getPose().getX(), null);
        builder.addDoubleProperty("Pose Y", () -> getPose().getY(), null);
        builder.addDoubleProperty("Pose Theta", () -> getPose().getRotation().getRadians(), null);

        builder.addDoubleProperty("Actual Speed X", () -> actualChassisSpeeds.vxMetersPerSecond, null);
        builder.addDoubleProperty("Actual Speed Y", () -> actualChassisSpeeds.vyMetersPerSecond, null);
        builder.addDoubleProperty("Actual Speed Theta", () -> actualChassisSpeeds.omegaRadiansPerSecond, null);
        builder.addBooleanProperty("Actually Moving", () -> isMoving(), null);

        builder.addDoubleProperty("Desired Speed X", () -> desiredChassisSpeeds.vxMetersPerSecond, null);
        builder.addDoubleProperty("Desired Speed Y", () -> desiredChassisSpeeds.vyMetersPerSecond, null);
    }
}
