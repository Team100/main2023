package frc.robot.autonomous;

import java.util.List;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * This is a simpler way to drive to a waypoint. It's just like
 * SwerveControllerCommand except that it generates the trajectory at the time
 * the command is scheduled, so it can capture the current robot location at
 * that instant. It runs forever, so it expects to be scheduled via
 * Trigger.whileTrue().
 */
public class DriveToWaypoint2 extends CommandBase {
    private static final TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(6, 12);

    private final Timer m_timer = new Timer();

    private final SwerveDriveSubsystem m_swerve;
    private final Pose2d goal;

    private final TrajectoryConfig translationConfig;
    private final ProfiledPIDController m_rotationController;
    private final PIDController xController;
    private final PIDController yController;
    private final HolonomicDriveController m_controller;

    private Trajectory m_trajectory;

    public DriveToWaypoint2(Pose2d goal, SwerveDriveSubsystem m_swerve) {
        this.goal = goal;
        this.m_swerve = m_swerve;
        m_rotationController = new ProfiledPIDController(1, 0, 0, rotationConstraints);
        m_rotationController.setTolerance(Math.PI/180);
        xController = new PIDController(1.2, 1.5, 0);
        xController.setIntegratorRange(-0.5, 0.1);
        xController.setTolerance(0.01);   
        yController = new PIDController(1.2, 1.5, 0);
        yController.setIntegratorRange(-0.5, 0.5);   
        yController.setTolerance(0.01);
        m_controller = new HolonomicDriveController(xController, yController, m_rotationController);
        translationConfig = new TrajectoryConfig(5.0, 20.0).setKinematics(SwerveDriveSubsystem.kDriveKinematics);
        addRequirements(m_swerve);
    }

    private Trajectory makeTrajectory() {
        Pose2d currentPose = m_swerve.getPose();
        Translation2d currentTranslation = currentPose.getTranslation();
        Translation2d goalTranslation = goal.getTranslation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();

        return TrajectoryGenerator.generateTrajectory(
                new Pose2d(currentTranslation, angleToGoal),
                List.of(),
                new Pose2d(goalTranslation, angleToGoal),
                translationConfig);
    }

    @Override
    public void initialize() {
        System.out.println("START TO WAYPOINT");
        m_timer.reset();
        m_timer.start();
        m_trajectory = makeTrajectory();
    }

    @Override
    public boolean isFinished() {
        return false; // keep trying until the button is released
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("END");
        m_timer.stop();
        
    }



    public void execute() {
        double curTime = m_timer.get();
        var desiredState = m_trajectory.sample(curTime);

        var targetChassisSpeeds = m_controller.calculate(m_swerve.getPose(), desiredState, goal.getRotation());
        var targetModuleStates = SwerveDriveSubsystem.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds);

        m_swerve.setModuleStates(targetModuleStates);
    }

}