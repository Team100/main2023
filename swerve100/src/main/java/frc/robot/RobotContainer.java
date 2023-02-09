package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autonomous.Circle;
import frc.robot.autonomous.MoveToAprilTag;
import frc.robot.autonomous.SanjanAutonomous;
import frc.robot.commands.ArmHigh;
import frc.robot.commands.ResetPose;
import frc.robot.commands.autoLevel;
import frc.robot.commands.driveLowerArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.SwerveDriveSubsystem;;

@SuppressWarnings("unused")
public class RobotContainer implements Sendable {
    // SUBSYSTEMS
    private final SwerveDriveSubsystem m_robotDrive;
    private final Manipulator manipulator;
    private final Arm arm;

    // CONTROLLERS
    private final XboxController controller0;
    private final XboxController controller1;

    // COMMANDS
    private final driveLowerArm driveLowerArm;
    private final ArmHigh armHigh;
    private final ResetPose resetPose;
    private final autoLevel autoLevel;
    private final MoveToAprilTag moveToAprilTag;

    public final static Field2d m_field = new Field2d();

    public RobotContainer() {
        // SUBSYSTEMS

        final double kDriveCurrentLimit = 30;
        // final double kDriveCurrentLimit = 50;
        m_robotDrive = new SwerveDriveSubsystem(kDriveCurrentLimit);
        manipulator = new Manipulator();
        arm = new Arm();

        // CONTROLLERS

        controller0 = new XboxController(0);
        controller1 = new XboxController(1);

        // COMMANDS

        // TODO: remove controller from commands
        driveLowerArm = new driveLowerArm(arm, controller1);
        armHigh = new ArmHigh(arm);
        resetPose = new ResetPose(m_robotDrive, new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
        autoLevel = new autoLevel(m_robotDrive.m_gyro, m_robotDrive);
        // TODO: do something with tagid.
        int tagID = 3;
        moveToAprilTag = new MoveToAprilTag(m_robotDrive, tagID);

        // TRIGGERS/BUTTONS

        // NOTE: per binding-commands-to-triggers.html, triggers/buttons can be
        // temporary, they're just to tell the scheduler what to do.

        // TODO: sort out which button should reset the pose
        // Left Bumper => Reset Pose
        new JoystickButton(controller0, XboxController.Button.kLeftBumper.value).onTrue(resetPose);
        // A button => Reset Pose
        new JoystickButton(controller0, XboxController.Button.kA.value).onTrue(resetPose);

        // B Button => Move to AprilTag
        new JoystickButton(controller0, XboxController.Button.kB.value).onTrue(moveToAprilTag);

        // TODO: sort out what the "Y" button should do
        // Y Button => Auto Level
        new JoystickButton(controller0, XboxController.Button.kY.value).onTrue(autoLevel);
        // Y Button => Sanjan Auton
        // new JoystickButton(controller0, XboxController.Button.kY.value).onTrue(new
        // SanjanAutonomous(m_robotDrive));

        // Controller 1 B Button => Arm high preset
        new JoystickButton(controller1, XboxController.Button.kB.value).onTrue(armHigh);

        // DEFAULT COMMANDS
        // Controller 0 right => cartesian, left => rotation
        m_robotDrive.setDefaultCommand(
                new RunCommand(
                        () -> {
                            final double kSpeedModifier = 1.0; // 0.5 (to go slower)
                            m_robotDrive.drive(
                                    -controller0.getRightY() * kSpeedModifier,
                                    -controller0.getRightX() * kSpeedModifier,
                                    -controller0.getLeftX() * kSpeedModifier,
                                    true);
                        },
                        m_robotDrive));

        // Controller 1 triggers => manipulator open/close
        manipulator
                .setDefaultCommand(
                        new RunCommand(
                                () -> manipulator.pinchv2(
                                        controller1.getRightTriggerAxis(),
                                        controller1.getLeftTriggerAxis()),
                                manipulator));
        // Controller 1 => arm motion
        arm.setDefaultCommand(driveLowerArm);


        SmartDashboard.putData("Robot Container", this);
    }

    // this is ishan's
    public Command getAutonomousCommand2() {
        // return new SequentialCommandGroup(
        // new IshanAutonomous(m_robotDrive),
        // new IshanAutonomous(m_robotDrive)
        // );

        // return new Forward(m_robotDrive, 5);
        return new Circle(m_robotDrive, 1.5);
    }

    public void runTest() {
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
        builder.addDoubleProperty("right y", () -> controller0.getRightY(), null);
        builder.addDoubleProperty("right x", () -> controller0.getRightX(), null);
        builder.addDoubleProperty("left x", () -> controller0.getLeftX(), null);
        builder.addDoubleProperty("theta controller error", () -> m_robotDrive.thetaController.getPositionError(),
                null);
        builder.addDoubleProperty("x controller error", () -> m_robotDrive.xController.getPositionError(), null);
        builder.addDoubleProperty("y controller error", () -> m_robotDrive.yController.getPositionError(), null);
    }
}
