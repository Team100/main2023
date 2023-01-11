// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve2DriveSubsystem;
//import frc.robot.subsystems.DriveSubsystem;

public class spin extends CommandBase {
  private Swerve2DriveSubsystem drive;
  /** Creates a new sin. */
  public spin(Swerve2DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


  }
  public SwerveControllerCommand traj(){
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
                  // Start at```  ` the origin facing the +X direction
                  new Pose2d(0, 0, new Rotation2d(0)),
                  // Pass through these two interior waypoints, making an 's' curve path
                  List.of(new Translation2d(5, 0), new Translation2d(5, 5), new Translation2d(0, 5)),
                  // End 3 meters straight ahead of where we started, facing forward
                  new Pose2d(0, 0, new Rotation2d(Math.PI)),
                  config);
    
          var thetaController =
              new ProfiledPIDController(
                  AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
          thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
          SwerveControllerCommand swerveControllerCommand =
              new SwerveControllerCommand(
                  exampleTrajectory,
                  drive::getPose, // Functional interface to feed supplier
                  Swerve2DriveSubsystem.kDriveKinematics,
    
                  // Position controllers
                  new PIDController(AutoConstants.kPXController, 0, 0),
                  new PIDController(AutoConstants.kPYController, 0, 0),
                  thetaController,
                  drive::setModuleStates,
                  drive);
    
          // Reset odometry to the starting pose of the trajectory.
          // drive.resetOdometry(exampleTrajectory.getInitialPose());
    
          // Run path following command, then stop at the end.
          return swerveControllerCommand;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
