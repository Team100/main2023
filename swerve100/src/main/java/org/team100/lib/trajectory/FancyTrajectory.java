// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.lib.trajectory;

import java.util.List;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import com.team254.frc2022.planners.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.CentripetalAccelerationConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class FancyTrajectory extends Command {
  /** Creates a new FancyTrajectory. */

  SwerveDriveSubsystem m_robotDrive;
  DriveMotionPlanner mMotionPlanner;
  public FancyTrajectory(SwerveDriveSubsystem robotDrive ) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_robotDrive = robotDrive;
    mMotionPlanner = new DriveMotionPlanner();

    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    final double kMaxVel = 1.0;
    final double kMaxAccel = 1.0;
    final double kMaxVoltage = 9.0;

    List<Pose2d> waypoints = List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(270)),
                new Pose2d(10, -10, Rotation2d.fromDegrees(0)));
        // while turning 180
        List<Rotation2d> headings = List.of(
                Rotation2d.fromDegrees(90),
                Rotation2d.fromDegrees(180));
        // these don't actually do anything.
        List<TimingConstraint<Pose2dWithCurvature>> constraints = List.of(
                new CentripetalAccelerationConstraint(60));

        // note there are static constraints in here.
        DriveMotionPlanner mMotionPlanner = new DriveMotionPlanner();
        boolean reversed = false;
        double start_vel = 0;
        double end_vel = 0;
        // there's a bug in here; it doesn't use the constraints, nor the voltage.
        Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> trajectory = mMotionPlanner
                .generateTrajectory(
                        reversed,
                        waypoints,
                        headings,
                        constraints,
                        start_vel,
                        end_vel,
                        kMaxVel,
                        kMaxAccel,
                        kMaxVoltage);
        System.out.println(trajectory);
        System.out.println("TRAJECTORY LENGTH: " + trajectory.length());
        // assertEquals(10, trajectory.length());

        TrajectoryIterator<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> iter = new TrajectoryIterator<>(
                new TimedView<>(trajectory));

        mMotionPlanner.setTrajectory(iter);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double now = Timer.getFPGATimestamp();
    
    Pose2d currentPose = new Pose2d(m_robotDrive.getPose().getX(), m_robotDrive.getPose().getY(), new Rotation2d(m_robotDrive.getPose().getRotation()));
    ChassisSpeeds output = mMotionPlanner.update(now, currentPose);

    m_robotDrive.setChassisSpeeds(output);

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
