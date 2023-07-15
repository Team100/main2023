package org.team100.frc2023.autonomous;

import java.util.List;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;

import org.team100.frc2023.commands.GoalOffset;
import org.team100.lib.sensors.RedundantGyro;
import org.team100.lib.subsystems.SwerveDriveSubsystem;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;

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

import org.team100.frc2023.LQRManager;
import org.team100.frc2023.commands.GoalOffset;
import org.team100.frc2023.subsystems.AHRSClass;
import org.team100.frc2023.subsystems.SwerveDriveSubsystem;
    
/**
 * This is a simpler way to drive to a waypoint. It's just like
 * SwerveControllerCommand except that it generates the trajectory at the time
 * the command is scheduled, so it can capture the current robot location at
 * that instant. It runs forever, so it expects to be scheduled via
 * Trigger.whileTrue().
 */


public class DriveToWaypoint3 extends CommandBase {
    private static final TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(8, 12);
    private final RedundantGyro m_gyro;
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
  
    private final LQRManager xManager;
    private final LQRManager yManager;
    private final HolonomicLQR m_controller;
 
    // private final Manipulator m_manipulator;

    // private Translation2d globalGoalTranslation;

    private Trajectory m_trajectory;
    private boolean isFinished = false;

    int count = 0;

    Matrix<N2, N2> matrix = new Matrix<>(Nat.N2(), Nat.N2());

    // Matrix<N2, N2> matrixB = new Matrix<>(Nat.N2(), Nat.N1());

    // matrixA.fill(0, 0, 0, 1, 0, 0)

    // LinearPlantInversionFeedforward<N2, N2, N1> feedforward = new LinearPlantInversionFeedforward<>(), count)

    // private State desiredStateGlobal;
  

       public DriveToWaypoint3(Pose2d goal, SwerveDriveSubsystem drivetrain, RedundantGyro gyro) {
        m_goal = goal;
        m_swerve = drivetrain;
        m_gyro = gyro;
        m_timer = new Timer();
        System.out.println("CONSTRUCTOR****************************************************");
         
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
         

        TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
          5,
          5);
          
        LinearSystem<N2, N1, N1> m_translationPlant = LinearSystemId.identifyPositionSystem(1.3 
        ,0.06);

        KalmanFilter<N2, N1, N1> m_translationObserver = new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            m_translationPlant,
            VecBuilder.fill(0.015, 0.17), // How accurate we
            // think our model is, in radians and radians/sec
            VecBuilder.fill(0.01), // How accurate we think our encoder position
            // data is. In this case we very highly trust our encoder position reading.
            0.020);

        LinearQuadraticRegulator<N2, N1, N1> m_translationController =
          new LinearQuadraticRegulator<>(
            m_translationPlant,
            VecBuilder.fill(0.05, 1), // qelms.
            VecBuilder.fill(20), // relms. Control effort (voltage) tolerance. Decrease this to more
            0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be

        
        xManager = new LQRManager(m_translationPlant, m_translationObserver, m_translationController, m_constraints);
        yManager = new LQRManager(m_translationPlant, m_translationObserver, m_translationController, m_constraints);
        
       // m_controller = new HolonomicDriveController2(xController, yController, m_rotationController, m_gyro);
        m_controller = new HolonomicLQR(m_swerve, xManager, yManager, m_rotationController, gyro);
        // m_controller = new HolonomicDriveController2(xController, yController, m_rotationController, m_gyro);
        
        translationConfig = new TrajectoryConfig(
                5, // velocity m/s
                5 // accel m/s/s
        ).setKinematics(SwerveDriveSubsystem.kDriveKinematics);

        // globalGoalTranslation = new Translation2d();

        // m_manipulator = manipulator;

        addRequirements(m_swerve);

        // SmartDashboard.putData("Drive To Waypoint", this);

  
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
        count = 0;
        m_controller.reset(m_swerve.getPose());
        m_controller.updateProfile(goal.getX(), goal.getY(), 5, 3, 1 );
        m_controller.start();
        // m_trajectory = makeTrajectory(previousOffset, 0);

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
        // if (m_trajectory == null) {
        //     return;
        // }
        // if (goalOffsetSupplier.get() != previousOffset) {
        //     m_trajectory = makeTrajectory(goalOffsetSupplier.get(),
        //             m_trajectory.sample(m_timer.get()).velocityMetersPerSecond);
        //     previousOffset = goalOffsetSupplier.get();
        //     m_timer.restart();
        // }
        // if (m_trajectory == null) {
        //     return;
        // }
        // double curTime = m_timer.get();
        // var desiredState = m_trajectory.sample(curTime);
        var targetChassisSpeeds = m_controller.calculate(m_swerve.getPose(), goal.getRotation());
        var targetModuleStates = SwerveDriveSubsystem.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds);
      
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

