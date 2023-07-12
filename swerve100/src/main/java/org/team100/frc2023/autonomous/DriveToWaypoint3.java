package org.team100.frc2023.autonomous;

import java.util.List;

import org.team100.frc2023.commands.GoalOffset;
import org.team100.frc2023.subsystems.AHRSClass;
import org.team100.frc2023.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToWaypoint3 extends CommandBase {
    private static final TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(8, 12);
    private final AHRSClass m_gyro;
    private final Pose2d m_goal;
    private final SwerveDriveSubsystem m_swerve;
    private final Timer m_timer;
    private final NetworkTableInstance inst;
    private final DoublePublisher desiredXPublisher;
    private final DoublePublisher desiredYPublisher;
    private final DoublePublisher poseXPublisher;
    private final DoublePublisher poseYPublisher;
    private final DoublePublisher poseRotPublisher;
    private final DoublePublisher desiredRotPublisher;
    private final DoublePublisher poseXErrorPublisher;
    private final DoublePublisher poseYErrorPublisher;
    private final DoublePublisher rotSetpoint;
    private final TrajectoryConfig translationConfig;
    private final ProfiledPIDController m_rotationController;
    private final PIDController xController;
    private final PIDController yController;
    private final HolonomicDriveController2 m_controller;
    private GoalOffset previousOffset;
    private Trajectory m_trajectory;
    private boolean isFinished = false;

    public DriveToWaypoint3(Pose2d goal, SwerveDriveSubsystem drivetrain, AHRSClass gyro) {
        m_goal = goal;
        m_swerve = drivetrain;
        m_gyro = gyro;
        m_timer = new Timer();

        inst = NetworkTableInstance.getDefault();
        desiredXPublisher = inst.getTable("Drive To Waypoint").getDoubleTopic("Desired X PUB").publish();
        desiredYPublisher = inst.getTable("Drive To Waypoint").getDoubleTopic("Desired Y PUB").publish();
        poseXPublisher = inst.getTable("Drive To Waypoint").getDoubleTopic("Pose X PUB").publish();
        poseYPublisher = inst.getTable("Drive To Waypoint").getDoubleTopic("Pose Y PUB").publish();
        poseRotPublisher = inst.getTable("Drive To Waypoint").getDoubleTopic("Pose Rot PUB").publish();
        desiredRotPublisher = inst.getTable("Drive To Waypoint").getDoubleTopic("Desired Rot PUB").publish();
        poseXErrorPublisher = inst.getTable("Drive To Waypoint").getDoubleTopic("Error X PUB").publish();
        poseYErrorPublisher = inst.getTable("Drive To Waypoint").getDoubleTopic("Error Y PUB").publish();
        rotSetpoint = inst.getTable("Drive To Waypoint").getDoubleTopic("Rot Setpoint").publish();

        m_rotationController = new ProfiledPIDController(6.5, 0, 1, rotationConstraints);
        m_rotationController.setTolerance(Math.PI / 180);

        xController = new PIDController(2, 0, 0);
        xController.setIntegratorRange(-0.3, 0.3);
        xController.setTolerance(0.00000001);

        yController = new PIDController(2, 0, 0);
        yController.setIntegratorRange(-0.3, 0.3);
        yController.setTolerance(0.00000001);

        m_controller = new HolonomicDriveController2(xController, yController, m_rotationController, m_gyro);

        translationConfig = new TrajectoryConfig(5, 4.5).setKinematics(SwerveDriveSubsystem.kDriveKinematics);
        addRequirements(drivetrain);
    }

    private Trajectory makeTrajectory(GoalOffset goalOffset, double startVelocity) {
        Pose2d currentPose = m_swerve.getPose();
        Translation2d currentTranslation = currentPose.getTranslation();
        Transform2d goalTransform = new Transform2d();

        Pose2d transformedGoal = m_goal.plus(goalTransform);

        Translation2d goalTranslation = transformedGoal.getTranslation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();
        TrajectoryConfig withStartVelocityConfig = new TrajectoryConfig(5, 2)
                .setKinematics(SwerveDriveSubsystem.kDriveKinematics);
        withStartVelocityConfig.setStartVelocity(startVelocity);

        try {
            return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(currentTranslation, angleToGoal),
                    List.of(),
                    new Pose2d(goalTranslation, angleToGoal),
                    translationConfig);
        } catch (TrajectoryGenerationException e) {
            isFinished = true;
            return null;
        }
    }

    @Override
    public void initialize() {
        isFinished = false;
        m_timer.restart();
        m_trajectory = makeTrajectory(previousOffset, 0);
    }

    @Override
    public boolean isFinished() {
        return isFinished; // keep trying until the button is released
    }

    @Override
    public void end(boolean interrupted) {
        // System.out.println("END");
        m_timer.stop();

    }

    public void execute() {
        if (m_trajectory == null) {
            return;
        }
        State desiredState = m_trajectory.sample(m_timer.get());
        ChassisSpeeds targetChassisSpeeds = m_controller.calculate(m_swerve.getPose(), desiredState,
                m_goal.getRotation());
        SwerveModuleState[] targetModuleStates = SwerveDriveSubsystem.kDriveKinematics
                .toSwerveModuleStates(targetChassisSpeeds);

        desiredXPublisher.set(desiredState.poseMeters.getX());
        desiredYPublisher.set(desiredState.poseMeters.getY());
        poseXPublisher.set(m_swerve.getPose().getX());
        poseYPublisher.set(m_swerve.getPose().getY());
        desiredRotPublisher.set(m_goal.getRotation().getRadians());
        rotSetpoint.set(m_rotationController.getSetpoint().position);
        poseRotPublisher.set(m_swerve.getPose().getRotation().getRadians());
        poseXErrorPublisher.set(xController.getPositionError());
        poseYErrorPublisher.set(yController.getPositionError());

        m_swerve.setModuleStates(targetModuleStates);
    }
}
