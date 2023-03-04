package frc.robot;

import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autonomous.Circle;
import frc.robot.autonomous.DriveToAprilTag;
import frc.robot.autonomous.DriveToWaypoint2;
import frc.robot.autonomous.MoveToAprilTag;
import frc.robot.autonomous.SanjanAutonomous;
import frc.robot.autonomous.VasiliAutonomous;
import frc.robot.commands.DriveRotation;
import frc.robot.commands.DriveWithHeading;
import frc.robot.commands.OpenManipulator;
import frc.robot.commands.ResetPose;
import frc.robot.commands.ResetRotation;
import frc.robot.commands.TimedClose;
import frc.robot.commands.autoLevel;
import frc.robot.commands.Arm.ArmTrajectory;
import frc.robot.commands.Arm.DriveToSetpoint;
import frc.robot.commands.Arm.ManualArm;
import frc.robot.commands.Manipulator.Close;
import frc.robot.commands.Manipulator.Home;
import frc.robot.commands.Manipulator.Open;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.Arm.ArmController;
import frc.robot.subsystems.Arm.ArmPosition;
import team100.commands.DriveManually;
import team100.commands.GripManually;
import team100.control.Control;
import team100.control.ControlSelect;
import team100.control.DualXboxControl;

@SuppressWarnings("unused")
public class RobotContainer implements Sendable {

    // THIS IS FROM BOB'S DELETED CODE

    // private static final XboxController m_driverController = new
    // XboxController(0);
    // private static final Joystick LEFT_JOYSTICK = new Joystick(0);
    private static final Joystick RIGHT_JOYSTICK = new Joystick(1);
    // private static final JoystickButton bButton = new
    // JoystickButton(m_driverController, 2);
    // private final Manipulator manipulator = new Manipulator();
    private boolean manipulatorCalibrated = false;
    // private int duration;
    // private double force;
    // private final Calibrate calibrate = new Calibrate(manipulator);
    // private final CloseManipulator closeManipulator = new CloseManipulator(manipulator);
    // private final OpenManipulator openManipulator = new OpenManipulator(manipulator);
    // private TimedClose timedClose = new TimedClose(manipulator, 300, 0.7);
    // private CurrentFeedbackClose currentFeedbackClose = new CurrentFeedbackClose(manipulator, 14.0, 0.5);

    // CONFIG
    private final DriverStation.Alliance m_alliance;

    // SUBSYSTEMS
    private final SwerveDriveSubsystem m_robotDrive;
    private final Manipulator manipulator;
    private final ArmController armController;

    // CONTROL
    private final Control control;

    // COMMANDS
    // private final ResetPose resetPose;
    private final autoLevel autoLevel;
    // private final MoveToAprilTag moveToAprilTag;
    private final DriveManually driveManually;
    private final GripManually gripManually;

    // private final DriveWithHeading driveWithHeading0, driveWithHeading90,
    // driveWithHeading180, driveWithHeading270;
    private final DriveWithHeading driveWithHeading;
    private final DriveRotation driveRotation;
    // private final DriveToAprilTag driveToAprilTag1;
    // private final DriveToAprilTag driveToAprilTag5;
    private final DriveToAprilTag driveToSubstation, driveToLeftGrid, driveToCenterGrid, driveToRightGrid;

    private final DriveToSetpoint highSetpoint;
    private final DriveToSetpoint highSafe;

    private final ArmTrajectory armHigh;
    private final ArmTrajectory armSafe;

    private final Open openManipulatorPID;
    private final Home homeCommand;
    private final Close closeCommand;

    // private final MoveToAprilTag moveToAprilTag2;

    public final static Field2d m_field = new Field2d();

    public RobotContainer() throws IOException {
        // THIS IS FROM BOB'S DELETED CODE

        final double kDriveCurrentLimit = 40;
        m_robotDrive = new SwerveDriveSubsystem(kDriveCurrentLimit);
        manipulator = new Manipulator();
        armController = new ArmController();
        manipulator.setDefaultCommand(new ConditionalCommand(
                new ConditionalCommand(
                        new CloseManipulator(manipulator, () -> !manipulator.getSensor()),
                        new RunCommand(() -> manipulator.pinch(-0.7 * RIGHT_JOYSTICK.getY()), manipulator),
                        () -> {
                            return false;
                        }),
                new Calibrate(manipulator).andThen(() -> {
                    manipulatorCalibrated = true;
                }),
                () -> {
                    return true;
                }));

        new JoystickButton(RIGHT_JOYSTICK, 2).onTrue(calibrate);
        new JoystickButton(RIGHT_JOYSTICK, 1).onTrue(closeManipulator);
        new JoystickButton(RIGHT_JOYSTICK, 3).onTrue(openManipulator);
        new JoystickButton(RIGHT_JOYSTICK, 4).onTrue(currentFeedbackClose);
        // new JoystickButton(RIGHT_JOYSTICK, 4).onTrue(timedClose);

        DigitalInput timeFlightSensor = new DigitalInput(3);
        Trigger tofTrigger = new Trigger(timeFlightSensor::get);
        tofTrigger.onFalse(currentFeedbackClose);

        // new JoystickButton(m_driverController, 1).onTrue(calibrate);
        // new JoystickButton(m_driverController, 8).onTrue(closeManipulator);

        // bButton.whenPressed(new RunCommand( () ->
        // manipulator.pinch(m_driverController.getLeftY())));
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        // new Trigger(m_exampleSubsystem::exampleCondition)
        // .onTrue(new ExampleCommand(m_exampleSubsystem));

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is
        // pressed,
        // cancelling on release.
        // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

        // SUBSYSTEMS
        final double kDriveCurrentLimit = 20;
        m_robotDrive = new SwerveDriveSubsystem(kDriveCurrentLimit);
        // manipulator = new Manipulator();
        arm = new Arm();

        // // NEW CONTROL
        control = ControlSelect.getControl();

        // COMMANDS
        // driveToAprilTag1 = DriveToAprilTag.newDriveToAprilTag(1, m_robotDrive);
        // driveToAprilTag5 = DriveToAprilTag.newDriveToAprilTag(4, m_robotDrive);
        m_alliance = DriverStation.getAlliance();
        if (m_alliance == DriverStation.Alliance.Blue) {
            driveToLeftGrid = DriveToAprilTag.newDriveToAprilTag(6, control::goalOffset, m_robotDrive);
            driveToCenterGrid = DriveToAprilTag.newDriveToAprilTag(7, control::goalOffset, m_robotDrive);
            driveToRightGrid = DriveToAprilTag.newDriveToAprilTag(8, control::goalOffset, m_robotDrive);
            driveToSubstation = DriveToAprilTag.newDriveToAprilTag(4, control::goalOffset, m_robotDrive);
        } else {
            driveToLeftGrid = DriveToAprilTag.newDriveToAprilTag(1, control::goalOffset, m_robotDrive);
            driveToCenterGrid = DriveToAprilTag.newDriveToAprilTag(2, control::goalOffset, m_robotDrive);
            driveToRightGrid = DriveToAprilTag.newDriveToAprilTag(3, control::goalOffset, m_robotDrive);
            driveToSubstation = DriveToAprilTag.newDriveToAprilTag(5, control::goalOffset, m_robotDrive);
        }


        highSetpoint = new DriveToSetpoint(armController, 0.91, 1.24, 0, 0);
        highSafe = new DriveToSetpoint(armController, 0.91, 0.59, -0.1, 0);
        
        armHigh = new ArmTrajectory( 1.24, 1.25, ArmPosition.high, armController);
        // armSafe = new ArmTrajectory(0.602, 0.404, armController);

        armSafe = new ArmTrajectory( 0.2, 0.125, ArmPosition.inward, armController);

        // moveToAprilTag2 = MoveToAprilTag.newMoveToAprilTag(m_robotDrive, 1);

        // m_reset = m_robotDrive.visionDataProvider.layout.getTagPose(5).get().toPose2d();
        // moveToAprilTag2 = MoveToAprilTag.newMoveToAprilTag(m_robotDrive, 1);

        armHigh = new ArmHigh(arm);
        // m_reset =
        // m_robotDrive.visionDataProvider.layout.getTagPose(5).get().toPose2d();
        ResetRotation resetRotation = new ResetRotation(m_robotDrive, new Rotation2d());
        autoLevel = new autoLevel(m_robotDrive.m_gyro, m_robotDrive);

        ResetPose resetPose = new ResetPose(m_robotDrive, 0, 0, 0);

        // driveWithHeading0 = new DriveWithHeading(
        // m_robotDrive,
        // control::xSpeed,
        // control::ySpeed,
        // () -> Rotation2d.fromDegrees(0),
        // "0 Degrees");

        // driveWithHeading90 = new DriveWithHeading(
        // m_robotDrive,
        // control::xSpeed,
        // control::ySpeed,
        // () -> Rotation2d.fromDegrees(90),
        // "90 Degrees");

        // driveWithHeading180 = new DriveWithHeading(
        // m_robotDrive,
        // control::xSpeed,
        // control::ySpeed,
        // () -> Rotation2d.fromDegrees(180),
        // "180 Degrees");

        driveWithHeading = new DriveWithHeading(
                m_robotDrive,
                control::xSpeed,
                control::ySpeed,
                control::desiredRotation,
                "");

        driveRotation = new DriveRotation(m_robotDrive, control::rotSpeed);

        // TODO: do something with tagid.
        int tagID = 5;
        // moveToAprilTag = MoveToAprilTag.newMoveToAprilTag(m_robotDrive, tagID);

        // TRIGGERS/BUTTXONS
        // control.driveToAprilTag(driveToAprilTag1);
        // control.driveToAprilTag2(driveToAprilTag1);
        control.resetRotation(resetRotation);
        control.autoLevel(autoLevel);
        // control.sanjanAuto(new SanjanAutonomous(m_robotDrive));
        control.resetPose(resetPose);
        // control.trajtoApril(moveToAprilTag2);
        control.driveToLeftGrid(driveToLeftGrid);
        control.driveToCenterGrid(driveToCenterGrid);
        control.driveToRightGrid(driveToRightGrid);
        control.driveToSubstation(driveToSubstation);
        control.resetPose(resetPose);

        control.driveToHigh(highSetpoint);
        control.driveToSafe(new SequentialCommandGroup(highSafe, new DriveToSetpoint(armController, 0.2, 0.125, 0, 0)));
        control.armHigh(armHigh);
        control.armSafe(armSafe);
        control.open(openManipulator);
        control.home(homeCommand);
        control.close(closeCommand);





        // DEFAULT COMMANDS
        // Controller 0 right => cartesian, left => rotation
        driveManually = new DriveManually(
                control::xSpeed,
                control::ySpeed,
                control::rotSpeed,
                m_robotDrive);
        // m_robotDrive.setDefaultCommand(driveManually);
        m_robotDrive.setDefaultCommand(driveWithHeading);

        // control.driveWithHeading0(driveWithHeading0);
        // control.driveWithHeading90(driveWithHeading90);
        // control.driveWithHeading180(driveWithHeading180);
        // control.driveWithHeading270(driveWithHeading270);
        control.driveRotation(driveRotation);

        // Controller 1 triggers => manipulator open/close
        gripManually = new GripManually(
                control::openSpeed,
                control::closeSpeed,
                manipulator);
        manipulator.setDefaultCommand(gripManually);

        armController.setDefaultCommand(manualArm);

        SmartDashboard.putData("Robot Container", this);
    }

    public Command getAutonomousCommand2() {
        // return new SequentialCommandGroup(
        // new IshanAutonomous(m_robotDrive),
        // new IshanAutonomous(m_robotDrive)
        // );

        return new VasiliAutonomous(m_robotDrive);

        // return new SanjanAutonomous(m_robotDrive);
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
        m_robotDrive.test(desiredOutputs);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("container");
        builder.addDoubleProperty("theta controller error", () -> m_robotDrive.thetaController.getPositionError(),
                null);
        builder.addDoubleProperty("x controller error", () -> m_robotDrive.xController.getPositionError(), null);
        builder.addDoubleProperty("y controller error", () -> m_robotDrive.yController.getPositionError(), null);

    }
}
