// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import javax.swing.plaf.basic.BasicBorders.ButtonBorder;

import org.opencv.video.SparsePyrLKOpticalFlow;

// import frc.robot.commands.spin;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand2;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoChargeLevel;
import frc.robot.localization.HashTag;
import frc.robot.localization.TestAprilTag;
import frc.robot.localization.resetPose;
// import frc.robot.commands.trajec;
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
   AutoChargeLevel autoChargeLevel = new AutoChargeLevel(m_robotDrive);
    private final resetPose ResetPose = new resetPose(m_robotDrive);

  // The driver's cont  roller
  private static final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private static final JoystickButton l2 = new JoystickButton(m_driverController, 9);
  private static final JoystickButton bButton = new JoystickButton(m_driverController, 2);
  // if true, test mode exercises module state (e.g. azimuth); if false, test mode
  // exercises module output directly.
 // boolean m_testModuleState = false;

  HashTag hashTag = new HashTag();


  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

          // An example trajectory to follow.  All units in meters.
    // Configure the button bindings
    configureButtonBindings(
    );

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->{
                    m_robotDrive.joystickDrive(
                    -m_driverController.getRightY()/2,
                    -m_driverController.getRightX()/2,
                    m_driverController.getLeftX()/2,
                    true);
                },
            m_robotDrive));


    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
    */
  private void configureButtonBindings() {
    bButton.whenPressed(ResetPose);


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


      Field2d m_field = new Field2d();
      SmartDashboard.putData(m_field);

      Pose2d aprilPose = new Pose2d(0, 0, new Rotation2d(0));
      // TestAprilTag aprilPose = new TestAprilTag(2, null) 
      Pose2d newRobotPose = new Pose2d(m_robotDrive.getPose().getX() + 10, m_robotDrive.getPose().getY(), m_robotDrive.getPose().getRotation() );
    
    // Trajectory exampleTrajectory2 =
    // TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direc  tion
    //     m_robotDrive.getPose(),
    //     // new Pose2d(0, 0, new Rotation2d(0)),
    //     List.of(),
    //     new Pose2d(aprilPose.getX() - 1, aprilPose.getY() , new Rotation2d(0)),
    //     // new Pose2d(8, 0, new Rotation2d(0)),
    //     // Pass config
    //     config);
      
      Trajectory exampleTrajectory =
      TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direc  tion
          // m_robotDrive.getPose(),
          m_robotDrive.getPose(),
          List.of(),
          // new Pose2d(aprilPose.getX() - 1, aprilPose.getY() , new Rotation2d(0)),
          newRobotPose,
          // Pass config
          config);

      
      SwerveControllerCommand2 swerveControllerCommand =
          new SwerveControllerCommand2(
              exampleTrajectory,
              m_robotDrive::getPose, // Functional interface to feed supplier
              Swerve2DriveSubsystem.kDriveKinematics,

              // Position controllers
              m_robotDrive.xController,
              m_robotDrive.yController,
              m_robotDrive.thetaController,
              () -> new Rotation2d(),
              m_robotDrive::setModuleStates,
              m_robotDrive);
      
              // m_field.getObject("traj").setTrajectory(exampleTrajectory);
      
              // System.out.println(exampleTrajectory);

        // SwerveControllerCommand2 swerveControllerCommand2 =
        //   new SwerveControllerCommand2(
        //       exampleTrajectory2,
        //       m_robotDrive::getPose, // Functional interface to feed supplier
        //       Swerve2DriveSubsystem.kDriveKinematics,

        //       // Position controllers
        //       m_robotDrive.xController,
        //       m_robotDrive.yController,
        //       m_robotDrive.thetaController,
        //       () -> new Rotation2d(),
        //       m_robotDrive::setModuleStates,
        //       m_robotDrive);
      
        //       // m_field.getObject("traj").setTrajectory(exampleTrajectory);
      
        //       System.out.println(exampleTrajectory2);


      // autonomous autonomous = new autonomous(m_robotDrive); 
      // return autonomous;

      return swerveControllerCommand;
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
    // System.out.println("GYYYYYYYYYYRPOOOOOOOOOOOOOOOOOOOOOO" + m_robotDrive.getHeading().getDegrees());
    // System.out.println("DEGREEEEEEEEEEEEES " + m_robotDrive.getPose().getRotation().getDegrees());
    m_robotDrive.resetAHRS2();
  }

  public void resetPose(){
    m_robotDrive.resetPose();
  }

}
