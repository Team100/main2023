package frc.robot.subsystems;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.SystemMenuBar;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.Nat;
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
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotPose;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
 

/**
 * This is a copy of DriveSubsystem for the second AndyMark swerve base.
 * 
 * It would be good to combine this with the DriveSubsystem somehow.
 */
public class Swerve2DriveSubsystem extends SubsystemBase {

    public static final double kTrackWidth = Constants.SwerveConstants.kTrackWidth;
    public static final double kWheelBase = Constants.SwerveConstants.kWheelBase;

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = true;

    public static final double ksVolts = 2;
    public static final double kvVoltSecondsPerMeter = 2.0;
    public static final double kaVoltSecondsSquaredPerMeter = 0.5;

    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = -5;
    public static double m_northOffset = 0;
    RobotPose robot = new RobotPose();

    private final SwerveModule m_frontLeft = SwerveModuleFactory
            .newSwerveModuleWithFalconDriveAndAnalogSteeringEncoders(
                    "Front Left",
                    11,
                    3, // motor
                    1, // encoder
                    false, // drive reverse
                    // false,
                    false, // steer encoder reverse
                    Constants.SwerveConstants.FRONT_LEFT_TURNING_OFFSET);
                    // OLD .51);
                    .97);


    private final SwerveModule m_frontRight = SwerveModuleFactory
            .newSwerveModuleWithFalconDriveAndAnalogSteeringEncoders(
                    "Front Right",
                    12,
                    1, // motor
                    3, // encoder
                    false, // drive reverse
                    false, // steer encoder reverse
                    Constants.SwerveConstants.FRONT_RIGHT_TURNING_OFFSET);
                    // OLD 0.54); // .54
                    0.04);

    private final SwerveModule m_rearLeft = SwerveModuleFactory
            .newSwerveModuleWithFalconDriveAndAnalogSteeringEncoders(
                    "Rear Left",
                    21,
                    2, // motor
                    0, // encoder
                    false, // drive reverse
                    false, // steer encoder reverse
                    Constants.SwerveConstants.REAR_LEFT_TURNING_OFFSET);
                    // OLD 0.74);
                    0.24);


    private final SwerveModule m_rearRight = SwerveModuleFactory
            .newSwerveModuleWithFalconDriveAndAnalogSteeringEncoders(
                    "Rear Right",
                    22,
                    0, // motor
                    2, // encoder
                    false, // drive reverse
                    false, // steer encoder reverse
                    Constants.SwerveConstants.REAR_RIGHT_TURNING_OFFSET);
                    // OLD .74); // .74
                    .24);


    // The gyro sensor. We have a Nav-X.
    private final AHRS m_gyro;
    // Odometry class for tracking robot pose
    SwerveDrivePoseEstimator<N7, N7, N5> m_poseEstimator;
    
    public PIDController xController = new PIDController(AutoConstants.kPXController, 0, 1);
    public PIDController yController = new PIDController(AutoConstants.kPYController, 0, 1);
    public ProfiledPIDController thetaController =
    new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0.5, AutoConstants.kThetaControllerConstraints);


    public ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    public PIDController xController = new PIDController(AutoConstants.kPXController, 0, 1);
    public PIDController yController = new PIDController(AutoConstants.kPYController, 0, 1);

    public Swerve2DriveSubsystem() {


        m_gyro = new AHRS(SerialPort.Port.kUSB);
        m_poseEstimator = new SwerveDrivePoseEstimator<N7, N7, N5>(
                Nat.N7(),
                Nat.N7(),
                Nat.N5(),
                m_gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                new Pose2d(),
                kDriveKinematics,
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.05, 0.05, 0.05, 0.05),
                VecBuilder.fill(Units.degreesToRadians(0.01), 0.01, 0.01, 0.01, 0.01),
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(1)));
        SmartDashboard.putData("Drive Subsystem", this);
    }

    public void updateOdometry() {
        m_poseEstimator.update(
            // OLD getHeading(),
            m_gyro.getRotation2d(),
            new SwerveModuleState[] {
              m_frontLeft.getState(),
              m_frontRight.getState(),
              m_rearLeft.getState(),
              m_rearRight.getState()
            },
            new SwerveModuleState[] {
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState()
            },
            new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_rearLeft.getPosition(),
              m_rearRight.getPosition()
            });

        // System.out.print(robot.getRobotPose(0));
        if (robot.aprilPresent()) {
            m_poseEstimator.addVisionMeasurement(
                    robot.getRobotPose(0),
                    Timer.getFPGATimestamp() - 0.3);
        }
    }

    @Override
    public void periodic() {
        updateOdometry();
    }

    public Pose2d getPose() {
        //System.out.println("YOOOOOOOOOOOOOOOOOOOOOOOOOO " + m_poseEstimator.getEstimatedPosition().getRotation());
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetPose(){
        m_poseEstimator.resetPosition(getHeading(), new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
      }, getPose());
    }

    // public void resetOdometry(Pose2d pose) {
    //     m_poseEstimator.resetPosition(pose, getHeading());
    //     m_poseEstimator.resetPosition(pose, Rotation2d.fromDegrees(-m_gyro.getFusedHeading()+m_northOffset));
    // }

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
                                // getHeading())
                                Rotation2d.fromDegrees(-m_gyro.getFusedHeading() + m_northOffset))
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
        // System.out.println("set outputs");
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

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
    }

    //public Rotation2d getHeading() {
    public double getHeading() {
        //return Rotation2d.fromDegrees((kGyroReversed ? -1.0 : 1.0) * m_gyro.getFusedHeading() + m_northOffset
        return Rotation2d.fromDegrees(-m_gyro.getFusedHeading() + m_northOffset).getDegrees();
        );
    }

    public double getTurnRate() {
        return m_gyro.getRate() * (kGyroReversed ? -1.0 : 1.0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("heading_degrees", () -> this.getHeading().getDegrees(), null);
        builder.addDoubleProperty("translationalx", () -> getPose().getX(), null);
        builder.addDoubleProperty("translationaly", () -> getPose().getY(), null);
        builder.addDoubleProperty("Front Left Position", () -> m_frontLeft.getPosition().distanceMeters, null ); 
        builder.addDoubleProperty("Front Right Position", () -> m_frontRight.getPosition().distanceMeters, null ); 
        builder.addDoubleProperty("Rear Left Position", () -> m_rearLeft.getPosition().distanceMeters, null ); 
        builder.addDoubleProperty("Rear Right Position", () -> m_rearRight.getPosition().distanceMeters, null ); 
        builder.addDoubleProperty("Theta Controller Error", () -> thetaController.getPositionError(), null ); 
        builder.addDoubleProperty("X Controller Error", () -> xController.getPositionError(), null );
        builder.addDoubleProperty("Y Controller Error", () -> yController.getPositionError(), null ); 
        builder.addDoubleProperty("estimatedtheta", () -> getPose().getRotation().getRadians(), null);
    }

    public void resetAHRS2() {
        m_northOffset = - getHeading().getDegrees();
        // m_gyro.reset();
    }

    public void autoChargeLevel(){
        drive(m_gyro.getAngle()/15, 0, 0, true);
    }
}
