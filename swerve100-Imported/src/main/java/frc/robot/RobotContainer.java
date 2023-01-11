// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import frc.robot.commands.spin;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.trajec;
import frc.robot.subsystems.Swerve2DriveSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements Sendable {
  // For the first andymark base
  //private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  // For the second andymark base
   private final Swerve2DriveSubsystem m_robotDrive = new Swerve2DriveSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
      final JoystickButton l2 = new JoystickButton(m_driverController, 9);
  // if true, test mode exercises module state (e.g. azimuth); if false, test mode
  // exercises module output directly.
 // boolean m_testModuleState = false;

  TrajectoryConfig config =
  new TrajectoryConfig(
         AutoConstants.kMaxSpeedMetersPerSecond,
         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
     // Add kinematics to ensure max speed is actually obeyed
     .setKinematics(Swerve2DriveSubsystem.kDriveKinematics);

  Trajectory exampleTrajectory =
    TrajectoryGenerator.generateTrajectory(
      // Start at```  ` the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(5, 0), new Translation2d(5, 5), new Translation2d(0, 5)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(0, 0, new Rotation2d(Math.PI)),
      config);
      

    private final ProfiledPIDController thetaController = new ProfiledPIDController( AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(exampleTrajectory, m_robotDrive::getPose, Swerve2DriveSubsystem.kDriveKinematics, new PIDController(AutoConstants.kPXController, 0, 0), new PIDController(AutoConstants.kPYController, 0, 0), thetaController, m_robotDrive::setModuleStates, m_robotDrive);


  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
          // An example trajectory to follow.  All units in meters.
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
                    true);
                },
            m_robotDrive));

            // new RunCommand(
            //   () ->{
            //           m_robotDrive.drive(
            //           0,
            //           0,
            //           0,
            //           true);
            //       },
            //   m_robotDrive));
            SmartDashboard.putData("Robot Container", this);

    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    l2.whileTrue(swerveControllerCommand);
  
  }

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
              .setKinematics(Swerve2DriveSubsystem.kDriveKinematics);

      // An example trajectory to follow.  All units in meters.
      Trajectory exampleTrajectory =
          TrajectoryGenerator.generateTrajectory(
              // Start at the origin facing the +X direction
              new Pose2d(0, 0, new Rotation2d(Math.PI)),new ArrayList<Translation2d> (), new Pose2d(1, 0, new Rotation2d(Math.PI)),
              config);

      var thetaController =
          new ProfiledPIDController(
              AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      SwerveControllerCommand swerveControllerCommand =
          new SwerveControllerCommand(
              exampleTrajectory,
              m_robotDrive::getPose, // Functional interface to feed supplier
              Swerve2DriveSubsystem.kDriveKinematics,

              // Position controllers
              new PIDController(AutoConstants.kPXController, 0, 0),
              new PIDController(AutoConstants.kPYController, 0, 0),
              thetaController,
              m_robotDrive::setModuleStates,
              m_robotDrive);

      // Reset odometry to the starting pose of the trajectory.
      //m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

      // Run path following command, then stop at the end.
      return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }

  public void runTest() {
    boolean rearLeft = m_driverController.getAButton();
    boolean rearRight = m_driverController.getBButton();
    boolean frontLeft = m_driverController.getXButton();
    boolean frontRight = m_driverController.getYButton();
    double driveControl = m_driverController.getLeftY();
    double turnControl = m_driverController.getLeftX();
    double[][] desiredOutputs = {
        {frontLeft?driveControl:0, frontLeft?turnControl:0},
        {frontRight?driveControl:0, frontRight?turnControl:0},
        {rearLeft?driveControl:0, rearLeft?turnControl:0},
      {rearRight?driveControl:0, rearRight?turnControl:0}
    };
    m_robotDrive.test(desiredOutputs);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("container");
    builder.addDoubleProperty("right y", () -> m_driverController.getRightY(), null);  
    builder.addDoubleProperty("right x", () -> m_driverController.getRightX(), null);
    builder.addDoubleProperty("left x", () -> m_driverController.getLeftX(), null);
  }
  public void resetAHRS() {
    m_robotDrive.resetAHRS2();
  }

}
