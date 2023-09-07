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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class FancyTrajectory extends Command {
  /** Creates a new FancyTrajectory. */

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable table = inst.getTable("Fancy Trajectory");
  private final DoublePublisher poseErrorX = table.getDoubleTopic("Pose Error X").publish();
  private final DoublePublisher poseErrorY = table.getDoubleTopic("Pose Error Y").publish();
  private final DoublePublisher velocityPublisher = table.getDoubleTopic("Velocity Setpoint").publish();


  
  SwerveDriveSubsystem m_robotDrive;
  DriveMotionPlanner mMotionPlanner;
  private TrajectoryIterator<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> mCurrentTrajectory;

  public FancyTrajectory(SwerveDriveSubsystem robotDrive ) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_robotDrive = robotDrive;
    mMotionPlanner = new DriveMotionPlanner();

    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    final double kMaxVel = 196;
    final double kMaxAccel = 196;
    final double kMaxVoltage = 9.0;

    List<Pose2d> waypoints = List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(90)),
                new Pose2d(80, 80, Rotation2d.fromDegrees(0)));
        // while turning 180
        List<Rotation2d> headings = List.of(
                Rotation2d.fromDegrees(0),
                Rotation2d.fromDegrees(0));
        // these don't actually do anything.
        List<TimingConstraint<Pose2dWithCurvature>> constraints = List.of(
                new CentripetalAccelerationConstraint(60));

        // note there are static constraints in here.
        // mMotionPlanner = new DriveMotionPlanner();
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


        mCurrentTrajectory = iter;
        mMotionPlanner.reset();
        mMotionPlanner.setTrajectory(iter);

        // if(mMotionPlanner.mCurrentTrajectory == null){
        //     System.out.println("BLLLLLLLLLLLLAJJJJJJJJJJJJJJJJjj");
        // }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double now = Timer.getFPGATimestamp();
    
    Pose2d currentPose = new Pose2d(  Units.metersToInches(m_robotDrive.getPose().getX()), Units.metersToInches(m_robotDrive.getPose().getY()), new Rotation2d(m_robotDrive.getPose().getRotation()));
    // if(mMotionPlanner.mCurrentTrajectory == null){
    //     System.out.println("AHHHHHHHHHHHHHHHHHHHHHHHHH");
    // }
    
    ChassisSpeeds output = mMotionPlanner.update(now, currentPose);


    poseErrorX.set(mMotionPlanner.getTranslationalError().x());
    poseErrorY.set(mMotionPlanner.getTranslationalError().y());
    velocityPublisher.set(mMotionPlanner.getVelocitySetpoint());

    
    // System.out.println("OUUUUUTPUUUUUUTTTTT XXXXXXXXXXXXXXXXXX" + output.vxMetersPerSecond);
    // System.out.println("OUUUUUTPUUUUUUTTTTT YYYYYYYYYYYYYYYYYY" + output.vyMetersPerSecond);

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
