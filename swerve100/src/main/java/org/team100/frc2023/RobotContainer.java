package org.team100.frc2023;

import java.io.FileWriter;
import java.io.IOException;

import org.team100.frc2023.autonomous.Autonomous;
import org.team100.frc2023.autonomous.DriveToAprilTag;
import org.team100.frc2023.autonomous.DriveToWaypoint3;
import org.team100.frc2023.autonomous.HolonomicDriveController2;
import org.team100.frc2023.autonomous.MoveConeWidth;
import org.team100.frc2023.autonomous.Rotate;
import org.team100.frc2023.commands.Defense;
import org.team100.frc2023.commands.DriveScaled;
import org.team100.frc2023.commands.DriveWithHeading;
import org.team100.frc2023.commands.RumbleOn;
import org.team100.frc2023.commands.arm.ArmTrajectory;
import org.team100.frc2023.commands.arm.ManualArm;
import org.team100.frc2023.commands.arm.SetConeMode;
import org.team100.frc2023.commands.arm.SetCubeMode;
import org.team100.frc2023.commands.manipulator.CloseSlow;
import org.team100.frc2023.commands.manipulator.Eject;
import org.team100.frc2023.commands.manipulator.Home;
import org.team100.frc2023.commands.retro.DriveToRetroReflectiveTape;
import org.team100.frc2023.control.Control;
import org.team100.frc2023.control.JoystickControl;
import org.team100.frc2023.subsystems.Manipulator;
import org.team100.frc2023.subsystems.arm.ArmPosition;
import org.team100.frc2023.subsystems.arm.ArmSubsystem;
import org.team100.lib.commands.ResetPose;
import org.team100.lib.commands.ResetRotation;
import org.team100.lib.commands.retro.LedOn;
import org.team100.lib.config.Identity;
import org.team100.lib.controller.DriveControllers;
import org.team100.lib.controller.DriveControllersFactory;
import org.team100.lib.indicator.LEDIndicator;
import org.team100.lib.indicator.LEDIndicator.State;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.VisionDataProvider;
import org.team100.lib.motion.drivetrain.Heading;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SpeedLimitsFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveLocal;
import org.team100.lib.motion.drivetrain.SwerveModuleCollection;
import org.team100.lib.motion.drivetrain.SwerveModuleCollectionFactory;
import org.team100.lib.motion.drivetrain.VeeringCorrection;
import org.team100.lib.motion.drivetrain.kinematics.FrameTransform;
import org.team100.lib.motion.drivetrain.kinematics.SwerveDriveKinematicsFactory;
import org.team100.lib.retro.Illuminator;
import org.team100.lib.sensors.RedundantGyro;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer implements Sendable {
    public static class Config {

        //////////////////////////////////////
        // SHOW MODE
        //
        // Show mode is for younger drivers to drive the robot slowly.
        //
        // TODO: make a physical show mode switch.
        public boolean SHOW_MODE = false;
        //
        //////////////////////////////////////

        public double kDriveCurrentLimit = SHOW_MODE ? 20 : 60;
    }

    private final Config m_config = new Config();

    // CONFIG
    private final DriverStation.Alliance m_alliance;

    // SUBSYSTEMS
    private final Heading m_heading;
    private final LEDIndicator m_indicator;
    private final RedundantGyro ahrsclass;
    private final Field2d m_field;
    private final AprilTagFieldLayoutWithCorrectOrientation layout;
    private final SwerveDriveSubsystem m_robotDrive;
    private final SwerveDriveKinematics m_kinematics;
    private final FrameTransform m_frameTransform;
    private final Manipulator manipulator;
    private final ArmSubsystem armController;
    private final Illuminator illuminator;

    // CONTROLLERS
    private final DriveControllers controllers;

    // HID CONTROL
    private final Control control;

    private final FileWriter myWriter;
    public static boolean enabled = false;
    public double m_routine = -1;

    public DriveToWaypoint3 driveLQR;

    public RobotContainer(DriverStation.Alliance alliance) throws IOException {
        // m_alliance = DriverStation.getAlliance();
        m_alliance = alliance;

        m_indicator = new LEDIndicator(8);
        ahrsclass = new RedundantGyro();
        m_heading = new Heading(ahrsclass);
        m_field = new Field2d();

        Identity identity = Identity.get();
        SpeedLimits speedLimits = SpeedLimitsFactory.get(identity, m_config.SHOW_MODE);
        m_kinematics = SwerveDriveKinematicsFactory.get(identity);

        VeeringCorrection veering = new VeeringCorrection(m_heading::getHeadingRateNWU);

        m_frameTransform = new FrameTransform(veering);

        SwerveModuleCollection modules = SwerveModuleCollectionFactory.get(identity, m_config.kDriveCurrentLimit);
        SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
                m_kinematics,
                m_heading.getHeadingNWU(),
                modules.positions(),
                new Pose2d(),
                VecBuilder.fill(0.5, 0.5, 0.5),
                VecBuilder.fill(0.4, 0.4, 0.4)); // note tight rotation variance here, used to be MAX_VALUE

        if (alliance == DriverStation.Alliance.Blue) {
            layout = AprilTagFieldLayoutWithCorrectOrientation.blueLayout();
        } else { // red
            layout = AprilTagFieldLayoutWithCorrectOrientation.redLayout();
        }

        VisionDataProvider visionDataProvider = new VisionDataProvider(
                layout,
                poseEstimator,
                poseEstimator::getEstimatedPosition);
        visionDataProvider.updateTimestamp(); // this is just to keep lint from complaining

        SwerveLocal swerveLocal = new SwerveLocal(speedLimits, m_kinematics, modules);
        
        controllers = new DriveControllersFactory().get(identity, speedLimits);
        HolonomicDriveController2 controller = new HolonomicDriveController2(
            controllers.xController,
            controllers.yController,
            controllers.thetaController);

        m_robotDrive = new SwerveDriveSubsystem(
                m_heading,
                poseEstimator,
                m_frameTransform,
                swerveLocal,
                controller,
                m_field);
        manipulator = new Manipulator();
        armController = new ArmSubsystem();
        illuminator = new Illuminator(25);

        // TODO: control selection using names
        // control = new DualXboxControl();
        control = new JoystickControl();

        myWriter = logFile();



        ////////////////////////////
        // DRIVETRAIN COMMANDS
        // control.autoLevel(new AutoLevel(false, m_robotDrive, ahrsclass));
        if (m_alliance == DriverStation.Alliance.Blue) {
            control.driveToLeftGrid(toTag(controller, 6, 1.25, 0));
            control.driveToCenterGrid(toTag(controller, 7, 0.95, .55));
            control.driveToRightGrid(toTag(controller, 8, 0.95, .55));
            control.driveToSubstation(toTag(controller, 4, 0.53, -0.749));
        } else {
            control.driveToLeftGrid(toTag(controller, 1, 0.95, .55));
            control.driveToCenterGrid(toTag(controller, 2, 0.95, .55));
            control.driveToRightGrid(toTag(controller, 3, 0.95, .55));
            control.driveToSubstation(toTag(controller, 5, 0.9, -0.72));
        }
        control.defense(new Defense(m_robotDrive));
        control.resetRotation0(new ResetRotation(m_robotDrive, new Rotation2d(0)));
        control.resetRotation180(new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)));
        control.driveSlow(new DriveScaled(control::twist, m_robotDrive, 0.4, 0.5));
        control.driveMedium(new DriveScaled(control::twist, m_robotDrive, 2.0, 0.5));
        control.resetPose(new ResetPose(m_robotDrive, 0, 0, 0));
        control.tapeDetect(new DriveToRetroReflectiveTape(m_robotDrive, m_frameTransform));
        control.rotate0(new Rotate(m_robotDrive, speedLimits, controllers.rotateController, new Timer(), 0));

        control.moveConeWidthLeft(new MoveConeWidth(m_robotDrive, 1));
        control.moveConeWidthRight(new MoveConeWidth(m_robotDrive, -1));

        control.driveWithLQR(new DriveToWaypoint3(new Pose2d(5, 0, new Rotation2d()), m_robotDrive, m_kinematics));

        ///////////////////////////
        // MANIPULATOR COMMANDS
        // control.open(new Open(manipulator));
        control.close(new Eject(manipulator));
        control.home(new Home(manipulator));
        control.closeSlow(new CloseSlow(manipulator));

        ////////////////////////////
        // ARM COMMANDS
        control.armHigh(new ArmTrajectory(ArmPosition.HIGH, armController, false));
        control.armSafe(new ArmTrajectory(ArmPosition.SAFE, armController, false));
        control.armSubstation(new ArmTrajectory(ArmPosition.SUB, armController, false));
        control.coneMode(new SetConeMode(armController, m_indicator));
        control.cubeMode(new SetCubeMode(armController, m_indicator));
        control.armLow(new ArmTrajectory(ArmPosition.MID, armController, false));
        control.armSafeBack(new ArmTrajectory(ArmPosition.SAFEBACK, armController, false));
        control.armToSub(new ArmTrajectory(ArmPosition.SUBTOCUBE, armController, false));
        control.safeWaypoint(new ArmTrajectory(ArmPosition.SAFEWAYPOINT, armController, false));
        // control.oscillate(new Oscillate(armController));
        control.oscillate(new ArmTrajectory(ArmPosition.SUB, armController, true));
        // control.armSafeSequential(armSafeWaypoint, armSafe);
        // control.armMid(new ArmTrajectory(ArmPosition.LOW, armController));

        //////////////////////////
        // MISC COMMANDS
        control.ledOn(new LedOn(illuminator));
        control.rumbleTrigger(new RumbleOn(control));

        ///////////////////////////
        // DRIVE

        if (m_config.SHOW_MODE) {
            m_robotDrive.setDefaultCommand(
                    new DriveScaled(
                            control::twist,
                            m_robotDrive,
                            speedLimits.kMaxSpeedMetersPerSecond,
                            speedLimits.kMaxAngularSpeedRadiansPerSecond));
        } else {
            m_robotDrive.setDefaultCommand(
                    new DriveWithHeading(
                            control::twist,
                            m_robotDrive,
                            controllers,
                            speedLimits.kMaxSpeedMetersPerSecond,
                            speedLimits.kMaxAngularSpeedRadiansPerSecond,
                            control::desiredRotation,
                            ahrsclass));
        }

        /////////////////////////
        // MANIPULATOR
        manipulator.setDefaultCommand(new RunCommand(() -> {
            manipulator.set(0, 30);
        }, manipulator));

        ////////////////////////
        // ARM
        armController.setDefaultCommand(new ManualArm(armController, control::lowerSpeed, control::upperSpeed));
        SmartDashboard.putData("Robot Container", this);
    }

    public Command getAutonomousCommand2(int routine) {
        return new Autonomous(
                m_robotDrive,
                m_frameTransform,
                armController,
                manipulator,
                ahrsclass,
                m_indicator,
                routine);
    }

    public void runTest() {
        XboxController controller0 = new XboxController(0);
        System.out.printf(
                "name: %s   left X: %5.2f   left Y: %5.2f   right X: %5.2f   right Y: %5.2f   left T: %5.2f   right T: %5.2f\n",
                DriverStation.getJoystickName(0),
                // DriverStation.getStickAxis(0, 0),
                controller0.getLeftX(), // 0 = GP right X, -0.66 to 0.83
                controller0.getLeftY(), // 1 = GP right Y, -0.64 to 0.64
                controller0.getRightX(), // 4 = GP left X, -0.7 to 0.8
                controller0.getRightY(), // 5
                controller0.getLeftTriggerAxis(), // 2 = GP left Y, -0.64 to 0.6
                controller0.getRightTriggerAxis() // 3
        );
    }

    public void runTest2() {

        XboxController controller0 = new XboxController(0);
        boolean rearLeft = controller0.getAButton();
        boolean rearRight = controller0.getBButton();
        boolean frontLeft = controller0.getXButton();
        boolean frontRight = controller0.getYButton();
        double driveControl = controller0.getLeftY();
        double turnControl = controller0.getLeftX();
        double[][] desiredOutputs = {
                { frontLeft ? driveControl : 0, frontLeft ? turnControl : 0 },
                { frontRight ? driveControl : 0, frontRight ? turnControl : 0 },
                { rearLeft ? driveControl : 0, rearLeft ? turnControl : 0 },
                { rearRight ? driveControl : 0, rearRight ? turnControl : 0 }
        };
        m_robotDrive.test(desiredOutputs, myWriter);

    }

    public static boolean isEnabled() {
        return enabled;
    }

    public boolean isBlueAlliance() {
        return m_alliance == DriverStation.Alliance.Blue;
    }

    public double getRoutine() {
        return m_routine;
    }

    public void ledStart() {
        m_indicator.set(State.ORANGE);
    }

    public void ledStop() {
        m_indicator.close();
    }

    public void red() {
        m_indicator.set(State.RED);
    }

    public void green() {
        m_indicator.set(State.GREEN);
    }

    private static FileWriter logFile() {
        try {
            return new FileWriter("/home/lvuser/logs.txt", true);
        } catch (IOException e) {
            e.printStackTrace();
        }
        return null;
    }

    private DriveToAprilTag toTag(
            HolonomicDriveController2 controller,
            int tagID,
            double xOffset,
            double yOffset) {
        return new DriveToAprilTag(
                tagID,
                xOffset,
                yOffset,
                control::goalOffset,
                m_robotDrive,
                m_kinematics,
                layout,
                () -> 0.0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("container");
        builder.addDoubleProperty("theta controller error",
                () -> controllers.thetaController.getPositionError(), null);
        builder.addDoubleProperty("x controller error",
                () -> controllers.xController.getPositionError(), null);
        builder.addDoubleProperty("y controller error",
                () -> controllers.yController.getPositionError(), null);
        builder.addBooleanProperty("Is Blue Alliance", () -> isBlueAlliance(), null);
        builder.addDoubleProperty("Routine", () -> getRoutine(), null);

        builder.addDoubleProperty("Theta Controller Error", () -> controllers.thetaController.getPositionError(), null);
        builder.addDoubleProperty("Theta Controller Setpoint", () -> controllers.thetaController.getSetpoint().position,
                null);

        builder.addDoubleProperty("X controller Error (m)", () -> controllers.xController.getPositionError(), null);
        builder.addDoubleProperty("X controller Setpoint", () -> controllers.xController.getSetpoint(), null);

        builder.addDoubleProperty("Y controller Error (m)", () -> controllers.yController.getPositionError(), null);
        builder.addDoubleProperty("Y controller Setpoint", () -> controllers.yController.getSetpoint(), null);

        builder.addDoubleProperty("Heading Degrees", () -> m_heading.getHeadingNWU().getDegrees(), null);
        builder.addDoubleProperty("Heading Radians", () -> m_heading.getHeadingNWU().getRadians(), null);

    }

}
