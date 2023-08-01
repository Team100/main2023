package org.team100.frc2023.autonomous;

import java.util.List;

import org.team100.frc2023.LQRManager;
import org.team100.frc2023.commands.GoalOffset;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
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
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This is a simpler way to drive to a waypoint. It's just like
 * SwerveControllerCommand except that it generates the trajectory at the time
 * the command is scheduled, so it can capture the current robot location at
 * that instant. It runs forever, so it expects to be scheduled via
 * Trigger.whileTrue().
 */

public class DriveToWaypoint3 extends Command {
    public static class Config {

        public TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(8, 12);
    }

    private final Config m_config = new Config();
    private final Pose2d m_goal;
    private final SwerveDriveSubsystem m_swerve;
    private final SwerveDriveKinematics m_kinematics;
    private final Timer m_timer;

    private final TrajectoryConfig translationConfig;
    private final ProfiledPIDController m_rotationController;

    private final PIDController xController;
    private final PIDController yController;
    private final HolonomicLQR m_controller;
    private GoalOffset previousOffset;
    private Trajectory m_trajectory;
    private boolean isFinished = false;

    private final LQRManager xManager;
    private final LQRManager yManager;

    // private final Manipulator m_manipulator;

    // private Translation2d globalGoalTranslation;

    int count = 0;

    Matrix<N2, N2> matrix = new Matrix<>(Nat.N2(), Nat.N2());

    // Matrix<N2, N2> matrixB = new Matrix<>(Nat.N2(), Nat.N1());

    // matrixA.fill(0, 0, 0, 1, 0, 0)

    // LinearPlantInversionFeedforward<N2, N2, N1> feedforward = new
    // LinearPlantInversionFeedforward<>(), count)

    // private State desiredStateGlobal;

    public DriveToWaypoint3(
            Pose2d goal,
            SwerveDriveSubsystem drivetrain,
            SwerveDriveKinematics kinematics) {
        m_goal = goal;
        m_swerve = drivetrain;
        m_kinematics = kinematics;

        m_timer = new Timer();

        m_rotationController = new ProfiledPIDController(6.5, 0, 1, m_config.rotationConstraints);
        m_rotationController.setTolerance(Math.PI / 180);

        xController = new PIDController(2, 0, 0);
        xController.setIntegratorRange(-0.3, 0.3);
        xController.setTolerance(0.00000001);

        yController = new PIDController(2, 0, 0);
        yController.setIntegratorRange(-0.3, 0.3);
        yController.setTolerance(0.00000001);

        TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
                5,
                5);

        LinearSystem<N2, N1, N1> m_translationPlant = LinearSystemId.identifyPositionSystem(1.3, 0.06);

        KalmanFilter<N2, N1, N1> m_translationObserver = new KalmanFilter<>(
                Nat.N2(),
                Nat.N1(),
                m_translationPlant,
                VecBuilder.fill(0.015, 0.17), // How accurate we
                // think our model is, in radians and radians/sec
                VecBuilder.fill(0.01), // How accurate we think our encoder position
                // data is. In this case we very highly trust our encoder position reading.
                0.020);

        LinearQuadraticRegulator<N2, N1, N1> m_translationController = new LinearQuadraticRegulator<>(
                m_translationPlant,
                VecBuilder.fill(0.05, 1), // qelms.
                VecBuilder.fill(20), // relms. Control effort (voltage) tolerance. Decrease this to more
                0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be

        xManager = new LQRManager(m_translationPlant, m_translationObserver, m_translationController, m_constraints);
        yManager = new LQRManager(m_translationPlant, m_translationObserver, m_translationController, m_constraints);

        // m_controller = new HolonomicDriveController2(xController, yController,
        // m_rotationController, m_gyro);
        m_controller = new HolonomicLQR(m_swerve, xManager, yManager, m_rotationController);
        // m_controller = new HolonomicDriveController2(xController, yController,
        // m_rotationController, m_gyro);

        // globalGoalTranslation = new Translation2d();

        // m_manipulator = manipulator;

        addRequirements(m_swerve);

        // SmartDashboard.putData("Drive To Waypoint", this);

        translationConfig = new TrajectoryConfig(5, 4.5).setKinematics(kinematics);
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

    @Override
    public void initialize() {
        isFinished = false;
        m_timer.restart();
        count = 0;
        m_controller.reset(m_swerve.getPose());
        m_controller.updateProfile(m_goal.getX(), m_goal.getY(), 5, 3, 1);
        m_controller.start();
        m_trajectory = makeTrajectory(previousOffset, 0);

    }

    public void execute() {
        // if (m_trajectory == null) {
        // return;
        // }
        // if (goalOffsetSupplier.get() != previousOffset) {
        // m_trajectory = makeTrajectory(goalOffsetSupplier.get(),
        // m_trajectory.sample(m_timer.get()).velocityMetersPerSecond);
        // previousOffset = goalOffsetSupplier.get();
        // m_timer.restart();
        // }
        // if (m_trajectory == null) {
        // return;
        // }
        double curTime = m_timer.get();
        State desiredState = m_trajectory.sample(curTime);

        Pose2d currentPose = m_swerve.getPose();
        Twist2d fieldRelativeTarget = m_controller.calculate(
                currentPose,
                desiredState,
                m_goal.getRotation());

        SwerveState manualState = SwerveDriveSubsystem.incremental(currentPose, fieldRelativeTarget);
        m_swerve.setDesiredState(manualState);

        desiredXPublisher.set(desiredState.poseMeters.getX());
        desiredYPublisher.set(desiredState.poseMeters.getY());
        poseXPublisher.set(m_swerve.getPose().getX());
        poseYPublisher.set(m_swerve.getPose().getY());
        desiredRotPublisher.set(m_goal.getRotation().getRadians());
        rotSetpoint.set(m_rotationController.getSetpoint().position);
        poseRotPublisher.set(m_swerve.getPose().getRotation().getRadians());
        poseXErrorPublisher.set(xController.getPositionError());
        poseYErrorPublisher.set(yController.getPositionError());
    }

    @Override
    public boolean isFinished() {
        return isFinished; // keep trying until the button is released
    }

    @Override
    public void end(boolean interrupted) {
        // System.out.println("END");
        m_timer.stop();
        m_swerve.truncate();
    }

    //////////////////////////////////////////////////////////////////////

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("drive to waypoint");
    private final DoublePublisher desiredXPublisher = table.getDoubleTopic("Desired X").publish();
    private final DoublePublisher desiredYPublisher = table.getDoubleTopic("Desired Y").publish();
    private final DoublePublisher poseXPublisher = table.getDoubleTopic("Pose X").publish();
    private final DoublePublisher poseYPublisher = table.getDoubleTopic("Pose Y").publish();
    private final DoublePublisher poseRotPublisher = table.getDoubleTopic("Pose Rot").publish();
    private final DoublePublisher desiredRotPublisher = table.getDoubleTopic("Desired Rot").publish();
    private final DoublePublisher poseXErrorPublisher = table.getDoubleTopic("Error X").publish();
    private final DoublePublisher poseYErrorPublisher = table.getDoubleTopic("Error Y").publish();
    private final DoublePublisher rotSetpoint = table.getDoubleTopic("Rot Setpoint").publish();

}
