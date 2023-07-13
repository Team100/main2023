package org.team100.frc2023.subsystems;

import java.io.FileWriter;
import java.io.IOException;

import org.team100.frc2023.RobotContainer;
import org.team100.frc2023.commands.DriveUtil;
import org.team100.lib.config.Identity;
import org.team100.lib.indicator.LEDIndicator;
import org.team100.lib.localization.VisionDataProvider;
import org.team100.lib.subsystems.DriveControllers;
import org.team100.lib.subsystems.DriveControllersFactory;
import org.team100.lib.subsystems.SpeedLimits;
import org.team100.lib.subsystems.SwerveDriveKinematicsFactory;
import org.team100.lib.subsystems.SwerveModuleCollection;
import org.team100.lib.subsystems.SwerveModuleCollectionFactory;

import edu.wpi.first.math.VecBuilder;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase {
    // TODO: calibrate this number.
    private static final double kVeeringCorrection = 0.15;

    public static final SwerveDriveKinematics kDriveKinematics = SwerveDriveKinematicsFactory.get(Identity.get());

    private final AHRSClass m_gyro;
    private final SpeedLimits speedLimits;
    public final DriveControllers controllers;
    private final SwerveModuleCollection m_modules;
    private final SwerveDrivePoseEstimator m_poseEstimator;

    private ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();

    public VisionDataProvider visionDataProvider;
    public final LEDIndicator m_indicator;

    // for observers
    private final DoubleArrayPublisher robotPosePub;
    private final StringPublisher fieldTypePub;
    private double xVelocity = 0;
    private double yVelocity = 0;
    private double thetaVelociy = 0;
    private boolean moving = false;

    // TODO: this looks broken
    public double keyList = -1;

    public SwerveDriveSubsystem(
            SpeedLimits speedLimits,
            DriverStation.Alliance alliance,
            double currentLimit,
            AHRSClass gyro,
            LEDIndicator indicator)
            throws IOException {
        m_gyro = gyro;
        m_indicator = indicator;

        // Sets up Field2d pose tracking for glass.
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable fieldTable = inst.getTable("field");
        robotPosePub = fieldTable.getDoubleArrayTopic("robotPose").publish();
        fieldTypePub = fieldTable.getStringTopic(".type").publish();
        fieldTypePub.set("Field2d");

        this.speedLimits = speedLimits;
        controllers = DriveControllersFactory.get(Identity.get(), speedLimits);
        m_modules = SwerveModuleCollectionFactory.get(Identity.get(), currentLimit);

        m_poseEstimator = new SwerveDrivePoseEstimator(
                kDriveKinematics,
                getHeading(),
                m_modules.positions(),
                new Pose2d(),
                VecBuilder.fill(0.5, 0.5, 0.5),
                VecBuilder.fill(0.4, 0.4, 0.4)); // note tight rotation variance here, used to be MAX_VALUE
        // VecBuilder.fill(0.01, 0.01, Integer.MAX_VALUE));
        visionDataProvider = new VisionDataProvider(alliance, m_poseEstimator, () -> getPose());
        SmartDashboard.putData("Drive Subsystem", this);
    }

    public void updateOdometry() {
        m_poseEstimator.update(
                getHeading(),
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
        m_poseEstimator.resetPosition(getHeading(), m_modules.positions(), robotPose);
    }

    public boolean getMoving() {
        return moving;
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
     * @param twist meters and radians per second
     */
    public void driveMetersPerSec(Twist2d twist, boolean fieldRelative) {
        double gyroRate = m_gyro.getRedundantGyroRate() * kVeeringCorrection;
        Rotation2d rotation2 = getPose().getRotation().minus(new Rotation2d(gyroRate));
        desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(twist.dx, twist.dy, twist.dtheta, rotation2);
        SwerveModuleState[] swerveModuleStates = kDriveKinematics.toSwerveModuleStates(fieldRelative
                ? desiredChassisSpeeds
                : new ChassisSpeeds(twist.dx, twist.dy, twist.dtheta));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, speedLimits.kMaxSpeedMetersPerSecond);
        getRobotVelocity(swerveModuleStates);
        m_modules.setDesiredStates(swerveModuleStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, speedLimits.kMaxSpeedMetersPerSecond);
        m_modules.setDesiredStates(desiredStates);
        getRobotVelocity(desiredStates);
    }

    public void setModuleStatesNoFF(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, speedLimits.kMaxSpeedMetersPerSecond);
        m_modules.setDesiredStatesNoFF(desiredStates);
        getRobotVelocity(desiredStates);
    }

    public ChassisSpeeds getRobotStates() {
        return m_modules.toChassisSpeeds(kDriveKinematics);
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

    public void test(double[][] desiredOutputs, FileWriter writer) {
        m_modules.test(desiredOutputs);
        try {
            if (writer != null) {
                writer.write("Timestamp: " + Timer.getFPGATimestamp() +
                        ", P" + m_modules.m_frontLeft.getPosition().distanceMeters
                        + ", " + m_modules.m_frontLeft.getState().speedMetersPerSecond + "\n");
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

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-m_gyro.getRedundantYaw());
    }

    public void stop() {
        m_modules.stop();
    }

    // for tests, to keep from conflicting when the indicator is created.
    public void close() {
        m_indicator.close();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        // Pose
        builder.addDoubleProperty("translationalx", () -> getPose().getX(), null);
        builder.addDoubleProperty("translationaly", () -> getPose().getY(), null);
        builder.addDoubleProperty("theta", () -> getPose().getRotation().getRadians(), null);

        builder.addDoubleProperty("Theta Controller Error", () -> controllers.thetaController.getPositionError(), null);
        builder.addDoubleProperty("Theta Controller Measurment", () -> getPose().getRotation().getRadians(), null);
        builder.addDoubleProperty("Theta Controller Setpoint", () -> controllers.thetaController.getSetpoint().position,
                null);

        builder.addDoubleProperty("X controller Error (m)", () -> controllers.xController.getPositionError(), null);
        builder.addDoubleProperty("X controller Setpoint", () -> controllers.xController.getSetpoint(), null);
        builder.addDoubleProperty("X controller Measurment", () -> getPose().getX(), null);

        builder.addDoubleProperty("Y controller Error (m)", () -> controllers.yController.getPositionError(), null);
        builder.addDoubleProperty("Y controller Setpoint", () -> controllers.yController.getSetpoint(), null);
        builder.addDoubleProperty("Y controller Measurment", () -> getPose().getY(), null);

        builder.addBooleanProperty("Moving", () -> getMoving(), null);

        builder.addDoubleProperty("X controller Velocity (m/s)", () -> xVelocity, null);
        builder.addDoubleProperty("Y controller Velocity (m/s)", () -> yVelocity, null);
        builder.addDoubleProperty("Theta controller Velocity (rad/s)", () -> thetaVelociy, null);

        builder.addDoubleProperty("Heading Degrees", () -> getHeading().getDegrees(), null);
        builder.addDoubleProperty("Heading Radians", () -> getHeading().getRadians(), null);

        builder.addDoubleProperty("ChassisSpeedDesired Odometry X (m/s)", () -> desiredChassisSpeeds.vxMetersPerSecond,
                null);
        builder.addDoubleProperty("ChassisSpeedDesired Odometry Y (m/s)", () -> desiredChassisSpeeds.vyMetersPerSecond,
                null);

        builder.addDoubleProperty("Heading Controller Setpoint (rad)",
                () -> controllers.headingController.getSetpoint().position,
                null);
        builder.addDoubleProperty("Heading Controller Measurment (rad)", () -> getPose().getRotation().getRadians(),
                null);
        builder.addDoubleProperty("Heading Controller Goal (rad)",
                () -> controllers.headingController.getGoal().position, null);
    }
}
