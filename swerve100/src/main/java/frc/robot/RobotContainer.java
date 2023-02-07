// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import javax.swing.plaf.basic.BasicBorders.ButtonBorder;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
// import frc.robot.commands.spin;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.autonomous.Circle;
import frc.robot.autonomous.Dodge;
import frc.robot.autonomous.Forward;
import frc.robot.autonomous.IshanAutonomous;
import frc.robot.autonomous.SanjanAutonomous;

import frc.robot.autonomous.MoveToAprilTag;
import frc.robot.commands.ResetPose;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.apriltag.AprilTagFields;

import frc.robot.commands.*;;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer implements Sendable {
  // For the first andymark base
  // private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  // For the second andymark base
  private final SwerveDriveSubsystem m_robotDrive = new SwerveDriveSubsystem();

  //Button Bindings
  private static final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  Trigger LB = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
  ///////////////////////
  //
  // TODO: resolve the use of joystick buttons here.
  JoystickButton Y = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  // Trigger yButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  //
  ///////////////////////
  
  JoystickButton bButton = new JoystickButton(m_driverController2, 2);
  public final static Field2d m_field = new Field2d();
  JoystickButton A = new JoystickButton(m_driverController, 1);
  JoystickButton B = new JoystickButton(m_driverController, 2);

  private static final XboxController m_driverController2 = new XboxController(1);
  // private static final JoystickButton bButton = new JoystickButton(m_driverController, 2);
  private final Manipulator manipulator = new Manipulator();

 private Arm arm = new Arm();

  private driveLowerArm driveLowerArm = new driveLowerArm(arm, m_driverController2);


  //Commands
  ResetPose resetPose = new ResetPose(m_robotDrive, new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
  autoLevel autoLevel = new autoLevel(m_robotDrive.m_gyro, m_robotDrive);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // An example trajectory to follow. All units in meters.
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.

        //Dont change these
        new RunCommand(
            () -> {
              m_robotDrive.drive(
////////////////////
//
// TODO: resolve the use of scaling here.
//                  -m_driverController.getRightY(),
//                  -m_driverController.getRightX(),
//                  -m_driverController.getLeftX(),
//                  false);
                  -m_driverController.getRightY() / 2,
                  -m_driverController.getRightX() / 2,
                  -m_driverController.getLeftX() / 2,
                  true);
            },
            m_robotDrive));

    SmartDashboard.putData("Robot Container", this);

    ArmHigh armHigh = new ArmHigh(arm);

    manipulator.setDefaultCommand(new RunCommand( () -> manipulator.pinchv2(m_driverController2.getRightTriggerAxis(), m_driverController2.getLeftTriggerAxis()), manipulator));
    arm.setDefaultCommand(driveLowerArm);

    bButton.onTrue(armHigh);


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
//////////////////////////
//
// TODO: resolve the use of buttons here
//
    LB.onTrue(resetPose);
    Y.onTrue(autoLevel);

    // LB.onTrue(resetPose);
    // Y.onTrue(new MoveToAprilTag(m_robotDrive, 3));
    Y.onTrue(new SanjanAutonomous(m_robotDrive));

    A.onTrue(resetPose);
    B.onTrue(new MoveToAprilTag(m_robotDrive, 3));


  }

  /**
   * 
   * Use this to pass the autonomous command to the main  {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  // this is ishan's
  public Command getAutonomousCommand2() {
    // return new SequentialCommandGroup(
    //   new IshanAutonomous(m_robotDrive),
    //   new IshanAutonomous(m_robotDrive)
    // );

    // return new Forward(m_robotDrive, 5);
    return new Circle(m_robotDrive, 1.5);
  }

  public void runTest() {
    boolean rearLeft = m_driverController.getAButton();
    boolean rearRight = m_driverController.getBButton();
    boolean frontLeft = m_driverController.getXButton();
    boolean frontRight = m_driverController.getYButton();
    double driveControl = m_driverController.getLeftY();
    double turnControl = m_driverController.getLeftX();
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
    builder.addDoubleProperty("right y", () -> m_driverController.getRightY(), null);
    builder.addDoubleProperty("right x", () -> m_driverController.getRightX(), null);
    builder.addDoubleProperty("left x", () -> m_driverController.getLeftX(), null);
    builder.addDoubleProperty("theta controller error", () -> m_robotDrive.thetaController.getPositionError(), null);
    builder.addDoubleProperty("x controller error", () -> m_robotDrive.xController.getPositionError(), null);
    builder.addDoubleProperty("y controller error", () -> m_robotDrive.yController.getPositionError(), null);
  }
}
