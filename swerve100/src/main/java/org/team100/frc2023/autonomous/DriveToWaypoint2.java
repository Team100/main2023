package org.team100.frc2023.autonomous;

import java.util.List;
import java.util.function.Supplier;

import org.team100.frc2023.commands.GoalOffset;
import org.team100.lib.sensors.RedundantGyro;
import org.team100.lib.subsystems.SwerveDriveSubsystem;
import org.team100.lib.subsystems.VeeringCorrection;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToWaypoint2 extends CommandBase {
    public static class Config {
        public TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(8, 12);
    }

    private final Config m_config = new Config();
    private final VeeringCorrection m_veering;
    private final Pose2d m_goal;
    private final SwerveDriveSubsystem m_swerve;
    private final SwerveDriveKinematics m_kinematics;
    private final Supplier<GoalOffset> m_goalOffsetSupplier;
    private final Supplier<Double> m_gamePieceOffsetSupplier;
    private final Timer m_timer;

    private final TrajectoryConfig translationConfig;
    private final ProfiledPIDController m_rotationController;
    private final PIDController xController;
    private final PIDController yController;
    private final HolonomicDriveController2 m_controller;
    private final double m_yOffset;
    private GoalOffset previousOffset;
    private Trajectory m_trajectory;
    private boolean isFinished = false;

    public DriveToWaypoint2(
            Pose2d goal,
            double yOffset,
            Supplier<GoalOffset> offsetSupplier,
            SwerveDriveSubsystem drivetrain,
            SwerveDriveKinematics kinematics,
            RedundantGyro gyro,
            Supplier<Double> gamePieceOffsetSupplier) {
        m_goal = goal;
        m_yOffset = yOffset;
        m_swerve = drivetrain;
        m_kinematics = kinematics;
        m_veering = new VeeringCorrection(gyro);
        m_goalOffsetSupplier = offsetSupplier;
        m_gamePieceOffsetSupplier = gamePieceOffsetSupplier;
        m_timer = new Timer();

        previousOffset = m_goalOffsetSupplier.get();

        m_rotationController = new ProfiledPIDController(6.5, 0, 1, m_config.rotationConstraints);
        m_rotationController.setTolerance(Math.PI / 180);

        xController = new PIDController(2, 0, 0);
        xController.setIntegratorRange(-0.3, 0.3);
        xController.setTolerance(0.00000001);

        yController = new PIDController(2, 0, 0);
        yController.setIntegratorRange(-0.3, 0.3);
        yController.setTolerance(0.00000001);

        m_controller = new HolonomicDriveController2(xController, yController, m_rotationController);

        translationConfig = new TrajectoryConfig(5, 4.5).setKinematics(kinematics);
        addRequirements(drivetrain);
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
        m_timer.stop();
    }

    public void execute() {
        if (m_trajectory == null) {
            return;
        }
        if (m_goalOffsetSupplier.get() != previousOffset) {
            m_trajectory = makeTrajectory(m_goalOffsetSupplier.get(),
                    m_trajectory.sample(m_timer.get()).velocityMetersPerSecond);
            previousOffset = m_goalOffsetSupplier.get();
            m_timer.restart();
        }
        if (m_trajectory == null) {
            return;
        }

        State desiredState = m_trajectory.sample(m_timer.get());

        desiredXPublisher.set(desiredState.poseMeters.getX());
        desiredYPublisher.set(desiredState.poseMeters.getY());
        desiredRotPublisher.set(m_goal.getRotation().getRadians());

        Pose2d currentPose = m_swerve.getPose();

        Twist2d fieldRelativeTarget = m_controller.calculate(
                currentPose,
                desiredState,
                m_goal.getRotation());

        m_swerve.driveInFieldCoords(fieldRelativeTarget);
    }

    ///////////////////////////////////////////////////////////////

    private Trajectory makeTrajectory(GoalOffset goalOffset, double startVelocity) {
        Pose2d currentPose = m_swerve.getPose();
        Translation2d currentTranslation = currentPose.getTranslation();
        Transform2d goalTransform = new Transform2d();
        if (goalOffset == GoalOffset.left) {
            goalTransform = new Transform2d(new Translation2d(0, -m_yOffset - m_gamePieceOffsetSupplier.get()),
                    new Rotation2d());
        }
        if (goalOffset == GoalOffset.right) {
            goalTransform = new Transform2d(new Translation2d(0, m_yOffset - m_gamePieceOffsetSupplier.get()),
                    new Rotation2d());
        }
        Pose2d transformedGoal = m_goal.plus(goalTransform);
        Translation2d goalTranslation = transformedGoal.getTranslation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();
        TrajectoryConfig withStartVelocityConfig = new TrajectoryConfig(5, 2).setKinematics(m_kinematics);
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

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // desired pose
    private final NetworkTable desired = inst.getTable("desired pose");
    private final DoublePublisher desiredXPublisher = desired.getDoubleTopic("x").publish();
    private final DoublePublisher desiredYPublisher = desired.getDoubleTopic("y").publish();
    private final DoublePublisher desiredRotPublisher = desired.getDoubleTopic("theta").publish();

}
