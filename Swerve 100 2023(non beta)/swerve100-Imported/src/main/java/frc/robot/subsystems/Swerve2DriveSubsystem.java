package frc.robot.subsystems;

import javax.swing.DebugGraphics;
import javax.swing.plaf.basic.BasicInternalFrameTitlePane.SystemMenuBar;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.localization.RobotPose;
// import frc.robot.localization.VisionDataProvider;
// import frc.robot.localization.VisionEstimate;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
 

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

    public static final double ksVolts = 2;
    public static final double kvVoltSecondsPerMeter = 2.0;
    public static final double kaVoltSecondsSquaredPerMeter = 0.5;

    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = -5;
    public static double m_northOffset = 0;
    boolean moving = false;

    RobotPose robot = new RobotPose();
    // VisionDataProvider vision = new VisionDataProvider();
    // VisionEstimate[] visionEstimates;
    

    private final SwerveModule m_frontLeft = SwerveModuleFactory
            .newSwerveModuleWithFalconDriveAndAnalogSteeringEncoders(
                    "Front Left",
                    11,
                    0, // motor
                    2, // encoder
                    false, // drive reverse
                    // false,
                    false, // steer encoder reverse
                    Constants.SwerveConstants.FRONT_LEFT_TURNING_OFFSET);
                    // .51);


    private final SwerveModule m_frontRight = SwerveModuleFactory
            .newSwerveModuleWithFalconDriveAndAnalogSteeringEncoders(
                    "Front Right",
                    12,
                    2, // motor
                    0, // encoder
                    false, // drive reverse
                    false, // steer encoder reverse
                    Constants.SwerveConstants.FRONT_RIGHT_TURNING_OFFSET);

                    // 0.54); // .54

    private final SwerveModule m_rearLeft = SwerveModuleFactory
            .newSwerveModuleWithFalconDriveAndAnalogSteeringEncoders(
                    "Rear Left",
                    21,
                    1, // motor
                    3, // encoder
                    false, // drive reverse
                    false, // steer encoder reverse
                    Constants.SwerveConstants.REAR_LEFT_TURNING_OFFSET);
                    // 0.74);


    private final SwerveModule m_rearRight = SwerveModuleFactory
            .newSwerveModuleWithFalconDriveAndAnalogSteeringEncoders(
                    "Rear Right",
                    22,
                    3, // motor
                    1, // encoder
                    false, // drive reverse
                    false, // steer encoder reverse
                    Constants.SwerveConstants.REAR_RIGHT_TURNING_OFFSET);
                    // .74); // .74


    // The gyro sensor. We have a Nav-X.
    private final AHRS m_gyro;
    // Odometry class for tracking robot pose
    SwerveDrivePoseEstimator m_poseEstimator;
    double pastVal = 0;
    double currVal = 0;

    double xVelocity = 0;
    double yVelocity = 0;

    private PIDController levelPid = new PIDController(1/60, 0, 0);
    public PIDController xController = new PIDController(AutoConstants.kPXController, AutoConstants.kIXController, AutoConstants.kDXController);
    public PIDController yController = new PIDController(AutoConstants.kPYController, AutoConstants.kIYController, AutoConstants.kDYController);
    public ProfiledPIDController thetaController =
    new ProfiledPIDController(
        AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);

    
    public Swerve2DriveSubsystem() {
        xController.setTolerance(0.3);
        yController.setTolerance(0.3);

        m_gyro = new AHRS(SerialPort.Port.kUSB);
        m_poseEstimator = new SwerveDrivePoseEstimator(
            kDriveKinematics,
            m_gyro.getRotation2d(),
            // getHeading(),
            new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_rearLeft.getPosition(),
              m_rearRight.getPosition()
            },
            new Pose2d());
        SmartDashboard.putData("Drive Subsystem", this);

    }

    public void updateOdometry() {
        m_poseEstimator.update(
            // getHeading(),
            m_gyro.getRotation2d(),
            // new SwerveModuleState[] {
            //   m_frontLeft.getState(),
            //   m_frontRight.getState(),
            //   m_rearLeft.getState(),
            //   m_rearRight.getState()
            // },
            new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_rearLeft.getPosition(),
              m_rearRight.getPosition()
            });
            // System.out.print(robot.getRobotPose(0));
            
            if(robot.aprilPresent() && !moving){
                // System.out.println("HIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII");
                Pose2d pose = robot.getRobotPose(0);
                if (pose != null)
                    m_poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp() - 0.3);
            } else {
                // System.out.println("NOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOoo");
                

            }
            

        }

    @Override
    public void periodic() {
        updateOdometry();
        // vision.periodic();
    }

    
    public Pose2d getPose() {
        //System.out.println("YOOOOOOOOOOOOOOOOOOOOOOOOOO " + m_poseEstimator.getEstimatedPosition().getRotation());
        return m_poseEstimator.getEstimatedPosition();

        // Translation2d translation2d = new Translation2d(m_poseEstimator.getEstimatedPosition().getX(), m_poseEstimator.getEstimatedPosition().getY());
        // Rotation2d rotation2d = new Rotation2d((m_poseEstimator.getEstimatedPosition().getRotation().getDegrees()));
        // Pose2d pose = new Pose2d(translation2d, rotation2d);
        // return pose;
    }

    // public void resetPose(){
    // m_poseEstimator.resetPosition(getHeading(), new SwerveModulePosition[] {
    //     m_frontLeft.getPosition(),
    //     m_frontRight.getPosition(),
    //     m_rearLeft.getPosition(),
    //     m_rearRight.getPosition()
    //   }, getPose());
    // }
      
    public void resetPose(){
        m_poseEstimator.resetPosition(m_gyro.getRotation2d(), new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          }, new Pose2d());
        }
          


    // public void resetOdometry(Pose2d pose) {
    //     m_poseEstimator.resetPosition(pose, getHeading());
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
        var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(kMaxSpeedMetersPerSecond * xSpeed,
                                kMaxSpeedMetersPerSecond * ySpeed, kMaxAngularSpeedRadiansPerSecond * rot,
                                getHeading())
                        : new ChassisSpeeds(kMaxSpeedMetersPerSecond * xSpeed, kMaxSpeedMetersPerSecond * ySpeed,
                                kMaxAngularSpeedRadiansPerSecond * rot));


        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, kMaxSpeedMetersPerSecond);


        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    public void joystickDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative){
        if (Math.abs(xSpeed) < .01)
            xSpeed = 100 * xSpeed * xSpeed * Math.signum(xSpeed);
        if (Math.abs(ySpeed) < .01)
            ySpeed = 100 * ySpeed * ySpeed * Math.signum(ySpeed);
        if (Math.abs(rot) < .1)
            rot = 0;

        drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    

    
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, kMaxSpeedMetersPerSecond);
        System.out.println("******************" + desiredStates);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);

        getRobotVelocity(desiredStates);


        
    }

    public void getRobotVelocity(SwerveModuleState[] desiredStates){
        // double frontLeft = Math.abs(m_frontLeft.getVelocity());
        // double frontRight = Math.abs(m_frontRight.getVelocity());
        // double rearLeft = Math.abs(m_rearLeft.getVelocity());
        // double rearRight = Math.abs(m_rearRight.getVelocity());

        // return (frontLeft + frontRight + rearLeft + rearRight) / 4;

        ChassisSpeeds chassisSpeeds = kDriveKinematics.toChassisSpeeds( desiredStates[0], desiredStates[1], desiredStates[2], desiredStates[3]);

        xVelocity = chassisSpeeds.vxMetersPerSecond;

        yVelocity = chassisSpeeds.vyMetersPerSecond;

        if(xVelocity > 0.1 || yVelocity > 0.1){
            moving = true;
        }

        

    }

    public void test(double[][] desiredOutputs) {
        //System.out.println("set outputs");
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

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-m_gyro.getFusedHeading() - m_northOffset);
    }

    public Rotation2d get180(){
        return Rotation2d.fromDegrees(-180);
    }

    public Rotation2d getReverseHeading() {
        double degrees = -getHeading().getDegrees();
        return Rotation2d.fromDegrees(degrees);
    }

    // public double getFusedHeading(){
    //     return -m_gyro.getFusedHeading();
    // }

    public double getTurnRate() {
        return m_gyro.getRate() * -1; //* (kGyroReversed ? -1.0 : 1.0);
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
        // builder.addDoubleProperty("Theta Controller Measurment", () -> thetaController, null ); 
        builder.addDoubleProperty("X Controller Error", () -> xController.getPositionError(), null );
        builder.addDoubleProperty("Y Controller Error", () -> yController.getPositionError(), null ); 
        builder.addDoubleProperty("Pose Estimator Radians", () -> getPose().getRotation().getRadians(), null ); 
        builder.addDoubleProperty("Pose Estimator Degrees", () -> getPose().getRotation().getDegrees(), null ); 


        builder.addDoubleProperty("Theta Controller Setpoint", () -> thetaController.getSetpoint().position, null ); 
        // builder.addDoubleProperty("Theta Controller Measurment", () -> thetaController, null ); 
        builder.addDoubleProperty("X Controller Setpoint", () -> xController.getSetpoint(), null );
        builder.addDoubleProperty("Y Controller Setpoint", () -> yController.getSetpoint(), null );

        builder.addDoubleProperty("Theta Controller Measurment", () -> thetaController.getSetpoint().position - thetaController.getPositionError(), null ); 
        // builder.addDoubleProperty("Theta Controller Measurment", () -> thetaController, null ); 
        builder.addDoubleProperty("X Controller Measurment", () -> xController.getSetpoint() - xController.getPositionError(), null );
        builder.addDoubleProperty("Y Controller Measurment", () -> yController.getSetpoint() - yController.getPositionError(), null );


        builder.addDoubleProperty("Gyro Roll", () -> m_gyro.getRoll(), null ); 
        builder.addDoubleProperty("Gyro Yaw", () -> m_gyro.getYaw(), null ); 
        // builder.addDoubleProperty("Gyro Pitch", () -> m_gyro.get(), null ); 
        builder.addDoubleProperty("Raw Gyro", () -> m_gyro.getFusedHeading(), null ); 
        builder.addDoubleProperty("Gyro Rotation2d", () -> m_gyro.getRotation2d().getDegrees(), null ); 
        builder.addDoubleProperty("Gyro 180", () -> get180().getDegrees(), null ); 

        // builder.addDoubleProperty("Robot Velocity", () -> getRobotVelocity(), null ); 
        builder.addDoubleProperty("Gyro 180", () -> get180().getDegrees(), null ); 

        builder.addBooleanProperty("Moving", () -> moving, null ); 

        // builder.addDoubleProperty("VISION ROBOT POSE X", () -> robot.getRobotPose(0).getX(), null ); 
        // builder.addDoubleProperty("VISION ROBOT POSE Y", () -> robot.getRobotPose(0).getY(), null ); 
        // builder.addDoubleProperty("VISION ROBOT POSE ROT", () -> robot.getRobotPose(0).getRotation().getDegrees(), null ); 

        // if (visionEstimates != null)      
        //     builder.addIntegerProperty("Visible April Tags", () -> visionEstimates.length, null);        

    }

    public void resetAHRS2() {
        m_northOffset = -m_gyro.getFusedHeading();
        // m_gyro.reset();
    }

    // public void autoChargeLevel(){
    //     double output = levelPid.calculate(m_gyro.getRoll());

    //     var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
    //             fieldRelative
    //                     ? ChassisSpeeds.fromFieldRelativeSpeeds(kMaxSpeedMetersPerSecond * xSpeed,
    //                             kMaxSpeedMetersPerSecond * ySpeed, kMaxAngularSpeedRadiansPerSecond * rot,
    //                             getHeading())
    //                     : new ChassisSpeeds(kMaxSpeedMetersPerSecond * xSpeed, kMaxSpeedMetersPerSecond * ySpeed,
    //                             kMaxAngularSpeedRadiansPerSecond * rot));

        



    
    // }

    public void autoChargeLevel(){

        currVal = m_gyro.getPitch();

        currVal = (pastVal + currVal)/2;

        drive(0, currVal / 60, 0, false);
        
        pastVal = currVal;

    }
}