// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  // if true, test mode exercises module state (e.g. azimuth); if false, test mode
  // exercises module output directly.
 // boolean m_testModuleState = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->{
                    m_robotDrive.drive(
                    m_driverController.getRightY(),
                    m_driverController.getRightX(),
                    m_driverController.getLeftX(),
                    false);
                },
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }

  //public void setTestState(boolean newState) {
  //  m_testModuleState = newState;
  //}

  public void runTest() {
    boolean rearLeft = m_driverController.getAButton();
    boolean rearRight = m_driverController.getBButton();
    boolean frontLeft = m_driverController.getXButton();
    boolean frontRight = m_driverController.getYButton();
//    double driveControl = m_driverController.getRightTriggerAxis();
//    double turnControl = m_driverController.getLeftTriggerAxis();
//    System.out.printf("%b %b %b %b %f %f\n",
//       rearLeft, rearRight, frontLeft, frontRight, driveControl, turnControl);
    double[][] desiredOutputs = {
        {frontLeft?1:0, frontLeft?1:0},
        {frontRight?1:0, frontRight?1:0},
        {rearLeft?1:0, rearLeft?1:0},
      {rearRight?1:0, rearRight?1:0}
    };
   // double[][] desiredOutputs = {
     //   {1, 1}, {0,0}, {0,0}, {0,0}
   // };
    m_robotDrive.test(desiredOutputs);
  }

  // Directly exercise the drivetrain.  Triggers control output (right
  // is drive, left is turn), A/B/X/Y buttons select modules.
  public Command getTestCommand() {

    return new RunCommand(() -> System.out.println("hi"));
    
  //  return new RunCommand(
  //      () ->{
  //          System.out.println("in test command");
            // right bumper button flips the test state.
            //if (m_driverController.getRightBumperPressed()) {
            //    setTestState(!m_testModuleState);
            //}
         //   boolean rearLeft = m_driverController.getAButton();
         //   boolean rearRight = m_driverController.getBButton();
         //   boolean frontLeft = m_driverController.getXButton();
         //   boolean frontRight = m_driverController.getYButton();
         //   double driveControl = m_driverController.getRightTriggerAxis();
         //   double turnControl = m_driverController.getLeftTriggerAxis();
            //if (m_testModuleState) {
            //    System.out.println("testing state");
            //    SwerveModuleState[] desiredStates = {
            //        new SwerveModuleState(frontLeft?driveControl:0,
            //                          Rotation2d.fromDegrees(360*(frontLeft?turnControl:0))),
            //        new SwerveModuleState(frontRight?driveControl:0,
            //                          Rotation2d.fromDegrees(360*(frontRight?turnControl:0))),
            //        new SwerveModuleState(rearLeft?driveControl:0,
            //                          Rotation2d.fromDegrees(360*(rearLeft?turnControl:0))),
            //        new SwerveModuleState(rearRight?driveControl:0,
            //                          Rotation2d.fromDegrees(360*(rearRight?turnControl:0)))
            //    };
            //    m_robotDrive.setModuleStates(desiredStates);
            //} else {
 //               System.out.println("testing output");
            //    double[][] desiredOutputs = {
            //        {frontLeft?driveControl:0, frontLeft?turnControl:0},
            //        {frontRight?driveControl:0, frontRight?turnControl:0},
            //        {rearLeft?driveControl:0, rearLeft?turnControl:0},
            //        {rearRight?driveControl:0, rearRight?turnControl:0}
            //    };
            //    m_robotDrive.test(desiredOutputs);
            //}
   //     },
     //   m_robotDrive);
  }
}
