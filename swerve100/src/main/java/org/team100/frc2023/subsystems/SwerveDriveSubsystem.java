package org.team100.frc2023.subsystems;

import java.io.FileWriter;
import java.io.IOException;

import org.team100.frc2023.RobotContainer;
import org.team100.frc2023.control.Control;
import org.team100.lib.config.Identity;
import org.team100.lib.indicator.LEDIndicator;
import org.team100.lib.localization.VisionDataProvider;
import org.team100.lib.subsystems.SwerveDriveKinematicsFactory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase {
    public static final SwerveDriveKinematics kDriveKinematics = SwerveDriveKinematicsFactory.get(Identity.get());
    // public ChassisSpeeds robotStates = new ChassisSpeeds();
    // public double observedVelocity;
    public ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();

    public final double kMaxSpeedMetersPerSecond;
    public final double kMaxAccelerationMetersPerSecondSquared;
    public final double kMaxAngularSpeedRadiansPerSecond;
    public final double kMaxAngularSpeedRadiansPerSecondSquared;

    // public final double kSlowSpeedMetersPerSecond;
    // public final double kSlowAngularSpeedRadiansPerSecond;

    public final SwerveModule m_frontLeft;
    public final SwerveModule m_frontRight;
    public final SwerveModule m_rearLeft;
    public final SwerveModule m_rearRight;

    // Odometry class for tracking robot pose
    private final SwerveDrivePoseEstimator m_poseEstimator;

    private double xVelocity = 0;
    private double yVelocity = 0;
    private double thetaVelociy = 0;

    public VisionDataProvider visionDataProvider;
    public final LEDIndicator indicator;

    private final AHRSClass m_gyro;

    private boolean moving = false;

    public final PIDController xController;
    public final PIDController yController;
    public final ProfiledPIDController headingController;
    public ProfiledPIDController thetaController;
    public final ProfiledPIDController rotateController;

    private final DoubleArrayPublisher robotPosePub;
    private final StringPublisher fieldTypePub;

    Control m_control;

    public double keyList = -1;

    public double m_xVelocity = 0;

    public SwerveDriveSubsystem(DriverStation.Alliance alliance, double currentLimit, AHRSClass gyro, Control control)
            throws IOException {
        m_gyro = gyro;
        // Sets up Field2d pose tracking for glass.
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable fieldTable = inst.getTable("field");
        robotPosePub = fieldTable.getDoubleArrayTopic("robotPose").publish();
        fieldTypePub = fieldTable.getStringTopic(".type").publish();
        fieldTypePub.set("Field2d");
        m_control = control;
        // kSlowSpeedMetersPerSecond = 0.5;
        // kSlowAngularSpeedRadiansPerSecond = 0.25;

        switch (Identity.get()) {
            case COMP_BOT:
                headingController = new ProfiledPIDController( //
                        0.67, // kP //0.75
                        0, // kI
                        0, // kD //0.1
                        new TrapezoidProfile.Constraints(
                                2 * Math.PI, // speed rad/s
                                4 * Math.PI)); // accel rad/s/s
                rotateController = new ProfiledPIDController( //
                        1, // kP //0.75
                        0.5, // kI
                        0, // kD //0.1
                        new TrapezoidProfile.Constraints(
                                2 * Math.PI, // speed rad/s
                                4 * Math.PI)); // accel rad/s/s
                headingController.setIntegratorRange(-0.1, 0.1);
                rotateController.setIntegratorRange(-0.2, 0.2);

                // Note very low heading tolerance.
                headingController.setTolerance(0.01);
                // THIS IS BABY MODE
                // kMaxSpeedMetersPerSecond = 1.5; //2
                // this is maximum maximum
                kMaxSpeedMetersPerSecond = 5;

                kMaxAccelerationMetersPerSecondSquared = 10; // 3
                kMaxAngularSpeedRadiansPerSecond = 5; // 5
                kMaxAngularSpeedRadiansPerSecondSquared = 5;
                xController = new PIDController(
                        0.15, // kP
                        0.0, // kI
                        0.0); // kD
                xController.setTolerance(0.01);

                yController = new PIDController(
                        0.15, // kP
                        0.0, // kI
                        0.0); // kD
                yController.setTolerance(0.01);

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
                        0.642472, // turn offset
                        currentLimit);
                m_frontRight = SwerveModuleFactory.WCPModule(
                        "Front Right",
                        12, // drive CAN
                        32, // turn CAN
                        1, // turn encoder
                        0.740540, // turn offset
                        currentLimit);
                m_rearLeft = SwerveModuleFactory.WCPModule(
                        "Rear Left",
                        21, // drive CAN
                        31, // turn CAN
                        2, // turn encoder
                        0.149713, // turn offset
                        currentLimit);
                m_rearRight = SwerveModuleFactory.WCPModule(
                        "Rear Right",
                        22, // drive CAN
                        33, // turn CAN
                        3, // turn encoder
                        0.296298, // turn offset
                        currentLimit);
                break;
            case SWERVE_TWO:
                headingController = new ProfiledPIDController( //
                        1, // kP
                        0, // kI
                        0.15, // kD
                        new TrapezoidProfile.Constraints(
                                2 * Math.PI, // speed rad/s
                                4 * Math.PI)); // accel rad/s/s

                rotateController = new ProfiledPIDController( //
                        0.7, // kP //0.75
                        0, // kI
                        0, // kD //0.1
                        new TrapezoidProfile.Constraints(
                                2 * Math.PI, // speed rad/s
                                4 * Math.PI)); // accel rad/s/s

                headingController.setIntegratorRange(-0.5, 0.5);
                // Note very low heading tolerance.
                headingController.setTolerance(Units.degreesToRadians(0.1));
                kMaxSpeedMetersPerSecond = 5;
                kMaxAccelerationMetersPerSecondSquared = 10;
                kMaxAngularSpeedRadiansPerSecond = 5;
                kMaxAngularSpeedRadiansPerSecondSquared = 5;

                xController = new PIDController(
                        2, // kP
                        0.1, // kI
                        0.0); // kD
                xController.setTolerance(0.01);
                xController.setIntegratorRange(-0.5, 0.5);

                yController = new PIDController(
                        2, // kP
                        0.1, // kI
                        0.0); // kD
                yController.setTolerance(0.01);
                yController.setIntegratorRange(-0.5, 0.5);

                thetaController = new ProfiledPIDController(
                        3, // kP
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
                        0.911606, // turn offset
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
                        0.871471, // turn offset
                        currentLimit);
                m_rearRight = SwerveModuleFactory.AMModule(
                        "Rear Right",
                        22, // drive CAN
                        0, // turn PWM
                        2, // turn encoder
                        0.605593, // turn offset
                        currentLimit);
                break;
            case SWERVE_ONE:
                headingController = new ProfiledPIDController( //
                        0.5, // kP
                        0, // kI
                        0, // kD
                        new TrapezoidProfile.Constraints(
                                2 * Math.PI, // speed rad/s
                                4 * Math.PI)); // accel rad/s/s

                rotateController = new ProfiledPIDController( //
                        0.7, // kP //0.75
                        0, // kI
                        0, // kD //0.1
                        new TrapezoidProfile.Constraints(
                                2 * Math.PI, // speed rad/s
                                4 * Math.PI)); // accel rad/s/s

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
            case BLANK: // for simulation; just like squarebot for now
                headingController = new ProfiledPIDController( //
                        1, // kP
                        .5, // kI
                        0.15, // kD
                        new TrapezoidProfile.Constraints(
                                2 * Math.PI, // speed rad/s
                                2 * Math.PI)); // accel rad/s/s

                rotateController = new ProfiledPIDController( //
                        0.7, // kP //0.75
                        0, // kI
                        0, // kD //0.1
                        new TrapezoidProfile.Constraints(
                                2 * Math.PI, // speed rad/s
                                4 * Math.PI)); // accel rad/s/s

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
            case CAMERA_DOLLY:
                headingController = new ProfiledPIDController( //
                        1, // kP
                        0, // kI
                        0, // kD
                        new TrapezoidProfile.Constraints(
                                2 * Math.PI, // speed rad/s
                                4 * Math.PI)); // accel rad/s/s

                rotateController = new ProfiledPIDController( //
                        0.7, // kP //0.75
                        0, // kI
                        0, // kD //0.1
                        new TrapezoidProfile.Constraints(
                                2 * Math.PI, // speed rad/s
                                4 * Math.PI)); // accel rad/s/s
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
                xController.setTolerance(0.01);

                yController = new PIDController(
                        0.15, // kP
                        0.0, // kI
                        0.0); // kD
                yController.setTolerance(0.01);

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
                        0.267276, // turn offset
                        currentLimit);
                m_frontRight = SwerveModuleFactory.WCPModule(
                        "Front Right",
                        12, // drive CAN
                        32, // turn CAN
                        1, // turn encoder
                        0.872709, // turn offset
                        currentLimit);
                m_rearLeft = SwerveModuleFactory.WCPModule(
                        "Rear Left",
                        21, // drive CAN
                        31, // turn CAN
                        2, // turn encoder
                        0.754813, // turn offset
                        currentLimit);
                m_rearRight = SwerveModuleFactory.WCPModule(
                        "Rear Right",
                        22, // drive CAN
                        33, // turn CAN
                        3, // turn encoder
                        0.477917, // turn offset
                        currentLimit);
                break;
            default:
                throw new IllegalStateException("Identity is not swerve: " + Identity.get().name());
        }
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
                VecBuilder.fill(0.5, 0.5, 0.5),
                VecBuilder.fill(0.4, 0.4, 0.4)); // note tight rotation variance here, used to be MAX_VALUE
        // VecBuilder.fill(0.01, 0.01, Integer.MAX_VALUE));
        visionDataProvider = new VisionDataProvider(alliance, m_poseEstimator, () -> getPose());
        indicator = new LEDIndicator(8);

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
        // System.out.println("MRRITRURUEUEUIWUIWEUIRHBIUEWFkj");
        updateOdometry();
        // m_control.rumbleOn();
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

    public void setKeyList() {
        keyList = NetworkTableInstance.getDefault().getTable("Vision").getKeys().size();
    }

    public double getVisionSize() {
        return keyList;
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
    public void drive(final double xSpeed, final double ySpeed, final double rot, boolean fieldRelative) {
        double xSpeedMetersPerSec = kMaxSpeedMetersPerSecond * MathUtil.clamp(xSpeed, -1, 1);
        double ySpeedMetersPerSec = kMaxSpeedMetersPerSecond * MathUtil.clamp(ySpeed, -1, 1);
        double rotRadiansPerSec = kMaxAngularSpeedRadiansPerSecond
                * MathUtil.applyDeadband(MathUtil.clamp(rot, -1, 1), 0.01);
        // if (Math.abs(xSpeed) < .01)
        // xSpeed = 100 * xSpeed * xSpeed * Math.signum(xSpeed);
        // if (Math.abs(ySpeed) < .01)
        // ySpeed = 100 * ySpeed * ySpeed * Math.signum(ySpeed);
        driveMetersPerSec(xSpeedMetersPerSec, ySpeedMetersPerSec, rotRadiansPerSec, fieldRelative);
    }

    public void driveTape(final double xSpeed, final double ySpeed, final double rot, boolean fieldRelative) {
        // double xSpeedMetersPerSec = kMaxSpeedMetersPerSecond * MathUtil.clamp(xSpeed,
        // -1, 1);
        // double ySpeedMetersPerSec = kMaxSpeedMetersPerSecond * MathUtil.clamp(ySpeed,
        // -1 ,1);
        // double rotRadiansPerSec = kMaxAngularSpeedRadiansPerSecond *
        // MathUtil.applyDeadband(MathUtil.clamp(rot, -1, 1), 0.01);

        driveMetersPerSec(xSpeed, ySpeed, rot, fieldRelative);
    }

    public void driveMetersPerSec(double xSpeedMetersPerSec, double ySpeedMetersPerSec, double rotRadiansPerSec,
            boolean fieldRelative) {
        double gyroRate = m_gyro.getRedundantGyroRate() * 0.15;
        // System.out.println(gyroRate);
        Rotation2d rotation2 = getPose().getRotation().minus(new Rotation2d(gyroRate));
        desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSec, ySpeedMetersPerSec,
                rotRadiansPerSec,
                rotation2);
        ChassisSpeeds startChassisSpeeds = new ChassisSpeeds(xSpeedMetersPerSec, ySpeedMetersPerSec, rotRadiansPerSec);
        var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? desiredChassisSpeeds
                        : startChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, kMaxSpeedMetersPerSecond);

        getRobotVelocity(swerveModuleStates);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    public void driveWithHeading(double xSpeedMetersPerSec, double ySpeedMetersPerSec, double rotRadiansPerSec,
            boolean fieldRelative) {
        double gyroRate = m_gyro.getRedundantGyroRate() * 0.15;
        double rotConstant = 0;
        // System.out.println(gyroRate);
        Rotation2d rotation2 = getPose().getRotation().minus(new Rotation2d(gyroRate));
        xSpeedMetersPerSec = xSpeedMetersPerSec
                * (Math.abs(Math.abs(rotConstant * rotRadiansPerSec / kMaxSpeedMetersPerSecond) - 1));
        ySpeedMetersPerSec = ySpeedMetersPerSec
                * (Math.abs(Math.abs(rotConstant * rotRadiansPerSec / kMaxSpeedMetersPerSecond) - 1));
        desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSec, ySpeedMetersPerSec,
                rotRadiansPerSec,
                rotation2);
        ChassisSpeeds startChassisSpeeds = new ChassisSpeeds(xSpeedMetersPerSec, ySpeedMetersPerSec, rotRadiansPerSec);
        var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? desiredChassisSpeeds
                        : startChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, kMaxSpeedMetersPerSecond);

        getRobotVelocity(swerveModuleStates);

        m_frontLeft.setDesiredDriveState(swerveModuleStates[0]);
        m_frontRight.setDesiredDriveState(swerveModuleStates[1]);
        m_rearLeft.setDesiredDriveState(swerveModuleStates[2]);
        m_rearRight.setDesiredDriveState(swerveModuleStates[3]);
    }

    public void driveSlow(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        xSpeed = MathUtil.clamp(xSpeed, -1, 1);
        ySpeed = MathUtil.clamp(ySpeed, -1, 1);
        rot = MathUtil.clamp(rot, -1, 1);
        // if (Math.abs(xSpeed) < .01)
        // xSpeed = 100 * xSpeed * xSpeed * Math.signum(xSpeed);

        // if (Math.abs(ySpeed) < .01)
        // ySpeed = 100 * ySpeed * ySpeed * Math.signum(ySpeed);
        if (Math.abs(rot) < .01)
            rot = 0;
        double gyroRate = m_gyro.getRedundantGyroRate() * 0.25;
        Rotation2d rotation2 = getPose().getRotation().minus(new Rotation2d(gyroRate));
        desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(6 * xSpeed,
                6 * ySpeed, 5 * rot,
                rotation2);
        var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? desiredChassisSpeeds
                        : new ChassisSpeeds(4.5 * xSpeed, 4.5 * ySpeed,
                                5 * rot));

        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, 4.5);

        getRobotVelocity(swerveModuleStates);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        // System.out.println(desiredChassisSpeeds);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    public void driveFine(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

        double kMaxSpeed = 0.8;
        double kMaxRot = 0.5;
        xSpeed = MathUtil.clamp(xSpeed, -1, 1);
        ySpeed = MathUtil.clamp(ySpeed, -1, 1);
        rot = MathUtil.clamp(rot, -1, 1);
        // if (Math.abs(xSpeed) < .01)
        // xSpeed = 100 * xSpeed * xSpeed * Math.signum(xSpeed);

        // if (Math.abs(ySpeed) < .01)
        // ySpeed = 100 * ySpeed * ySpeed * Math.signum(ySpeed);
        if (Math.abs(rot) < .01)
            rot = 0;
        double gyroRate = m_gyro.getRedundantGyroRate() * 0.25;
        Rotation2d rotation2 = getPose().getRotation().minus(new Rotation2d(gyroRate));
        desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(kMaxSpeed * xSpeed,
                kMaxSpeed * ySpeed, kMaxRot * rot,
                rotation2);
        var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? desiredChassisSpeeds
                        : new ChassisSpeeds(kMaxSpeed * xSpeed, kMaxSpeed * ySpeed,
                                kMaxRot * rot));

        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, kMaxSpeed);

        getRobotVelocity(swerveModuleStates);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    public void driveMedium(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

        double kMaxSpeed = 3.5;
        double kMaxRot = 0.5;
        xSpeed = MathUtil.clamp(xSpeed, -1, 1);
        ySpeed = MathUtil.clamp(ySpeed, -1, 1);
        rot = MathUtil.clamp(rot, -1, 1);
        // if (Math.abs(xSpeed) < .01)
        // xSpeed = 100 * xSpeed * xSpeed * Math.signum(xSpeed);

        // if (Math.abs(ySpeed) < .01)
        // ySpeed = 100 * ySpeed * ySpeed * Math.signum(ySpeed);
        if (Math.abs(rot) < .01)
            rot = 0;
        double gyroRate = m_gyro.getRedundantGyroRate() * 0.25;
        Rotation2d rotation2 = getPose().getRotation().minus(new Rotation2d(gyroRate));
        desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(kMaxSpeed * xSpeed,
                kMaxSpeed * ySpeed, kMaxRot * rot,
                rotation2);
        var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? desiredChassisSpeeds
                        : new ChassisSpeeds(kMaxSpeed * xSpeed, kMaxSpeed * ySpeed,
                                kMaxRot * rot));

        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, kMaxSpeed);

        getRobotVelocity(swerveModuleStates);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        // System.out.println(desiredChassisSpeeds);
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

    public void setModuleStatesNoFF(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredStateNoFF(desiredStates[0]);
        m_frontRight.setDesiredStateNoFF(desiredStates[1]);
        m_rearLeft.setDesiredStateNoFF(desiredStates[2]);
        m_rearRight.setDesiredStateNoFF(desiredStates[3]);

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

        m_xVelocity = xVelocity;

        if (xVelocity >= 0.1 || yVelocity >= 0.1 || thetaVelociy >= 0.1) {
            moving = true;
        } else {
            moving = false;
        }
    }

    public void test(double[][] desiredOutputs, FileWriter writer) {
        m_frontLeft.setOutput(desiredOutputs[0][0], desiredOutputs[0][1]);
        m_frontRight.setOutput(desiredOutputs[1][0], desiredOutputs[1][1]);
        m_rearLeft.setOutput(desiredOutputs[2][0], desiredOutputs[2][1]);
        m_rearRight.setOutput(desiredOutputs[3][0], desiredOutputs[3][1]);

        // full-speed printing to see the signal without networktables
        // System.out.printf("T %5.3f FL(p%5.3f v%5.3f) FR(p%5.3f v%5.3f) RL(p%5.3f
        // v%5.3f) RR(p%5.3f v%5.3f)\n",
        // System.out.printf("T %5.3f FL(p%5.3f v%5.3f)",

        try {
            if (writer != null) {
                writer.write("Timestamp: " + Timer.getFPGATimestamp() + ", P" + m_frontLeft.getPosition().distanceMeters
                        + ", " + m_frontLeft.getState().speedMetersPerSecond + "\n");
                // writer.write("Deez");

                writer.flush();
                // System.out.println("Successfully wrote to the file.");
            }
        } catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }

        // Timer.getFPGATimestamp(),
        // m_frontLeft.getPosition().distanceMeters,
        // m_frontLeft.getState().speedMetersPerSecond
        // m_frontRight.getPosition().distanceMeters,
        // m_frontRight.getState().speedMetersPerSecond,
        // m_rearLeft.getPosition().distanceMeters,
        // m_rearLeft.getState().speedMetersPerSecond,
        // m_rearRight.getPosition().distanceMeters,
        // m_rearRight.getState().speedMetersPerSecond

    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetDriveEncoders();
        m_frontRight.resetDriveEncoders();
        m_rearLeft.resetDriveEncoders();
        m_rearRight.resetDriveEncoders();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-m_gyro.getRedundantYaw());
    }

    // for tests, to keep from conflicting when the indicator is created.
    public void close() {
        indicator.close();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        // Pose
        builder.addDoubleProperty("translationalx", () -> getPose().getX(), null);
        builder.addDoubleProperty("translationaly", () -> getPose().getY(), null);
        builder.addDoubleProperty("theta", () -> getPose().getRotation().getRadians(), null);

        builder.addDoubleProperty("Theta Controller Error", () -> thetaController.getPositionError(), null);
        builder.addDoubleProperty("Theta Controller Measurment", () -> getPose().getRotation().getRadians(), null);
        builder.addDoubleProperty("Theta Controller Setpoint", () -> thetaController.getSetpoint().position, null);

        builder.addDoubleProperty("X controller Error (m)", () -> xController.getPositionError(), null);
        builder.addDoubleProperty("X controller Setpoint", () -> xController.getSetpoint(), null);
        builder.addDoubleProperty("X controller Measurment", () -> getPose().getX(), null);

        builder.addDoubleProperty("Y controller Error (m)", () -> yController.getPositionError(), null);
        builder.addDoubleProperty("Y controller Setpoint", () -> yController.getSetpoint(), null);
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

        builder.addDoubleProperty("Heading Controller Setpoint (rad)", () -> headingController.getSetpoint().position,
                null);
        builder.addDoubleProperty("Heading Controller Measurment (rad)", () -> getPose().getRotation().getRadians(),
                null);
        builder.addDoubleProperty("Heading Controller Goal (rad)", () -> headingController.getGoal().position, null);

    }
}
