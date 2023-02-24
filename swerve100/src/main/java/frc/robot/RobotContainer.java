package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autonomous.Circle;
import frc.robot.autonomous.DriveToAprilTag;
import frc.robot.autonomous.DriveToWaypoint2;
import frc.robot.autonomous.MoveToAprilTag;
import frc.robot.autonomous.SanjanAutonomous;
import frc.robot.autonomous.VasiliAutonomous;
import frc.robot.commands.ArmHigh;
import frc.robot.commands.DriveRotation;
import frc.robot.commands.DriveWithHeading;
import frc.robot.commands.ResetPose;
import frc.robot.commands.ResetRotation;
import frc.robot.commands.autoLevel;
import frc.robot.commands.driveLowerArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.SwerveDriveSubsystem;
import team100.commands.DriveManually;
import team100.commands.GripManually;
import team100.control.Control;
import team100.control.ControlSelect;
import team100.control.DualXboxControl;

@SuppressWarnings("unused")
public class RobotContainer implements Sendable {
    // SUBSYSTEMS
    private final SwerveDriveSubsystem m_robotDrive;
    private final Manipulator manipulator;
    private final Arm arm;

    // CONTROL
    private final Control control;

    // COMMANDS
    private final driveLowerArm driveLowerArm;
    private final ArmHigh armHigh;
    // private final ResetPose resetPose;
    private final autoLevel autoLevel;
    // private final MoveToAprilTag moveToAprilTag;
    private final DriveManually driveManually;
    private final GripManually gripManually;

    private final DriveWithHeading driveWithHeading0, driveWithHeading90, driveWithHeading180, driveWithHeading270;
    private final DriveRotation driveRotation;
    private final DriveToAprilTag driveToAprilTag1;
    private final DriveToAprilTag driveToAprilTag5;
    private final DriveToWaypoint2 driveToID4, driveToID3, driveToID2, driveToID1;

    // private final MoveToAprilTag moveToAprilTag2;

    public final static Field2d m_field = new Field2d();

    public RobotContainer() {
        // SUBSYSTEMS

        final double kDriveCurrentLimit = 20;
        m_robotDrive = new SwerveDriveSubsystem(kDriveCurrentLimit);
        manipulator = new Manipulator();
        arm = new Arm();

        // // NEW CONTROL
        control = ControlSelect.getControl();

        // COMMANDS
        driveToAprilTag1 = DriveToAprilTag.newDriveToAprilTag(1, m_robotDrive);
        driveToAprilTag5 = DriveToAprilTag.newDriveToAprilTag(4, m_robotDrive);

        driveToID1 = new DriveToWaypoint2(new Pose2d(14.513558, 1.071626, new Rotation2d(0)), m_robotDrive);
        driveToID2 = new DriveToWaypoint2(new Pose2d(14.513558, 2.748026, new Rotation2d(0)), m_robotDrive);
        driveToID3 = new DriveToWaypoint2(new Pose2d(14.513558, 4.424426, new Rotation2d(0)), m_robotDrive);
        driveToID4 = new DriveToWaypoint2(new Pose2d(15.178784, 6.749796, new Rotation2d(0)), m_robotDrive);
        
        // moveToAprilTag2 = MoveToAprilTag.newMoveToAprilTag(m_robotDrive, 1);

        armHigh = new ArmHigh(arm);
        // m_reset = m_robotDrive.visionDataProvider.layout.getTagPose(5).get().toPose2d();
        ResetRotation resetRotation = new ResetRotation(m_robotDrive, new Rotation2d());
        autoLevel = new autoLevel(m_robotDrive.m_gyro, m_robotDrive);

        ResetPose resetPose = new ResetPose(m_robotDrive, 0, 0, 0);
        
        driveWithHeading0 = new DriveWithHeading(
            m_robotDrive, 
            control::xSpeed,
            control::ySpeed,
            Rotation2d.fromDegrees(0),
            "0 Degrees");

        driveWithHeading90 = new DriveWithHeading(
            m_robotDrive, 
            control::xSpeed,
            control::ySpeed,
            Rotation2d.fromDegrees(90),
            "90 Degrees");

        driveWithHeading180 = new DriveWithHeading(
            m_robotDrive, 
            control::xSpeed,
            control::ySpeed,
            Rotation2d.fromDegrees(180),
            "180 Degrees");
        
        driveWithHeading270 = new DriveWithHeading(
            m_robotDrive, 
            control::xSpeed,
            control::ySpeed,
            Rotation2d.fromDegrees(270),
            "270 Degrees");
        
        driveRotation = new DriveRotation(m_robotDrive, control::rotSpeed);
        
        // TODO: do something with tagid.
        int tagID = 5;
        // moveToAprilTag = MoveToAprilTag.newMoveToAprilTag(m_robotDrive, tagID);

        // TRIGGERS/BUTTXONS
        control.driveToAprilTag(driveToAprilTag1);
        control.driveToAprilTag2(driveToAprilTag1);
        control.resetRotation(resetRotation);
        control.autoLevel(autoLevel);
        // control.sanjanAuto(new SanjanAutonomous(m_robotDrive));
        control.armHigh(armHigh);
        control.resetPose(resetPose);
        // control.trajtoApril(moveToAprilTag2);
        control.driveToID1(driveToID1);
        control.driveToID2(driveToID2);
        control.driveToID3(driveToID3);
        control.driveToID4(driveToID4);





        // DEFAULT COMMANDS
        // Controller 0 right => cartesian, left => rotation
        driveManually = new DriveManually(
                control::xSpeed,
                control::ySpeed,
                control::rotSpeed,
                m_robotDrive);
        m_robotDrive.setDefaultCommand(driveManually);
        
        control.driveWithHeading0(driveWithHeading0);
        control.driveWithHeading90(driveWithHeading90);
        control.driveWithHeading180(driveWithHeading180);
        control.driveWithHeading270(driveWithHeading270);
        control.driveRotation(driveRotation);




        // Controller 1 triggers => manipulator open/close
        gripManually = new GripManually(
                control::openSpeed,
                control::closeSpeed,
                manipulator);
        manipulator.setDefaultCommand(gripManually);

        // Controller 1 => arm motion
        driveLowerArm = new driveLowerArm(
                control::lowerSpeed,
                control::upperSpeed,
                arm);
        arm.setDefaultCommand(driveLowerArm);

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
