package team100.frc2023.autonomous;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team100.frc2023.commands.GoalOffset;
import team100.frc2023.subsystems.AHRSClass;
import team100.frc2023.subsystems.SwerveDriveSubsystem;
    
/**
 * This is a simpler way to drive to a waypoint. It's just like
 * SwerveControllerCommand except that it generates the trajectory at the time
 * the command is scheduled, so it can capture the current robot location at
 * that instant. It runs forever, so it expects to be scheduled via
 * Trigger.whileTrue().
 */
public class DriveToWaypoint2 extends CommandBase {
    private static final TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(8, 12);
    private final AHRSClass m_gyro;
    private double desiredX = 0;
    private double desiredY = 0;
    // private Pose2d desiredPose;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    DoublePublisher desiredXPublisher = inst.getTable("Drive To Waypoint").getDoubleTopic("Desired X PUB").publish();
    DoublePublisher desiredYPublisher = inst.getTable("Drive To Waypoint").getDoubleTopic("Desired Y PUB").publish();
   
    DoublePublisher poseXPublisher = inst.getTable("Drive To Waypoint").getDoubleTopic("Pose X PUB").publish();
    DoublePublisher poseYPublisher = inst.getTable("Drive To Waypoint").getDoubleTopic("Pose Y PUB").publish();
    DoublePublisher poseRotPublisher = inst.getTable("Drive To Waypoint").getDoubleTopic("Pose Rot PUB").publish();
    DoublePublisher desiredRotPublisher = inst.getTable("Drive To Waypoint").getDoubleTopic("Desired Rot PUB").publish();

    DoublePublisher poseXErrorPublisher = inst.getTable("Drive To Waypoint").getDoubleTopic("Error X PUB").publish();
    DoublePublisher poseYErrorPublisher = inst.getTable("Drive To Waypoint").getDoubleTopic("Error Y PUB").publish();

    DoublePublisher rotSetpoint = inst.getTable("Drive To Waypoint").getDoubleTopic("Rot Setpoint").publish();

    // DoublePublisher holonomicYSetpoint = inst.getTable("Drive To Waypoint").getDoubleTopic("Holonomic Y Setpoint").publish();
    // DoublePublisher holonomicXSetpoint = inst.getTable("Drive To Waypoint").getDoubleTopic("Holonomic X Setpoint").publish();

    // DoublePublisher holonomicYMeasurment = inst.getTable("Drive To Waypoint").getDoubleTopic("Holonomic Y Measurment").publish();
    // DoublePublisher holonomicXMeasurment= inst.getTable("Drive To Waypoint").getDoubleTopic("Holonomic X Measurment").publish();

    private final Timer m_timer = new Timer();

    private final SwerveDriveSubsystem m_swerve;
    private final Pose2d goal;
    private final Supplier<GoalOffset> goalOffsetSupplier;
    private final Supplier<Double> m_gamePieceOffsetSupplier;

    private final double m_yOffset;
    private GoalOffset previousOffset;
    private Transform2d goalTransform;

    private final TrajectoryConfig translationConfig;
    private final ProfiledPIDController m_rotationController;
    private final PIDController xController;
    private final PIDController yController;
    private final HolonomicDriveController2 m_controller;

    // private final Manipulator m_manipulator;

    // private Translation2d globalGoalTranslation;

    private Trajectory m_trajectory;
    private boolean isFinished = false;

    int count = 0;

    // private State desiredStateGlobal;

    public DriveToWaypoint2(Pose2d goal, double yOffset, Supplier<GoalOffset> offsetSupplier,
            SwerveDriveSubsystem m_swerve, AHRSClass gyro, Supplier<Double> gamePieceOffsetSupplier) {
        this.goal = goal;
        this.m_swerve = m_swerve;
        m_gyro = gyro;
        System.out.println("CONSTRUCTOR****************************************************");

        goalOffsetSupplier = offsetSupplier;
        m_gamePieceOffsetSupplier = gamePieceOffsetSupplier;

        previousOffset = goalOffsetSupplier.get();
        m_yOffset = yOffset;

        m_rotationController = new ProfiledPIDController(6.5, 0, 1, rotationConstraints); //4.5
        m_rotationController.setTolerance(Math.PI / 180);

        // xController = new PIDController(2, 0, 0);

        xController = new PIDController(2, 0, 0); //2.5
        xController.setIntegratorRange(-0.3, 0.3);
        xController.setTolerance(0.00000001);

        yController = new PIDController(2, 0, 0); //2.5
        yController.setIntegratorRange(-0.3, 0.3);
        yController.setTolerance(0.00000001);

        // yController.setTolerance(0.05);

        m_controller = new HolonomicDriveController2(xController, yController, m_rotationController, m_gyro);
        
        translationConfig = new TrajectoryConfig(
                5, // velocity m/s
                4.5 // accel m/s/s
        ).setKinematics(SwerveDriveSubsystem.kDriveKinematics);

        // globalGoalTranslation = new Translation2d();

        // m_manipulator = manipulator;

        addRequirements(m_swerve);

        // SmartDashboard.putData("Drive To Waypoint", this);

    }

    private Trajectory makeTrajectory(GoalOffset goalOffset, double startVelocity) {
        Pose2d currentPose = m_swerve.getPose();
        Translation2d currentTranslation = currentPose.getTranslation();
        goalTransform = new Transform2d();
        // TODO: Change based on task
        if (goalOffset == GoalOffset.left) {

            goalTransform = new Transform2d(new Translation2d(0, -m_yOffset - m_gamePieceOffsetSupplier.get()), new Rotation2d());
            System.out.println("lalallalalalalalalalalalalallalalalala");
        }
        if (goalOffset == GoalOffset.right) {
            goalTransform = new Transform2d(new Translation2d(0, m_yOffset - m_gamePieceOffsetSupplier.get()), new Rotation2d());
            System.out.println("fffffffffffffffffffffffffffffffffffffffffffff");
        }
        Pose2d transformedGoal = goal.plus(goalTransform);
        System.out.println(goalOffset);
        Translation2d goalTranslation = transformedGoal.getTranslation();
        Translation2d translationToGoal = goalTranslation.minus(currentTranslation);
        Rotation2d angleToGoal = translationToGoal.getAngle();
        TrajectoryConfig withStartVelocityConfig = new TrajectoryConfig(5, 2)
                .setKinematics(SwerveDriveSubsystem.kDriveKinematics);
        withStartVelocityConfig.setStartVelocity(startVelocity);

        // globalGoalTranslation = goalTranslation;
        // TODO: Change starting waypoint to align with starting velocity
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
        if (goalOffsetSupplier.get() != previousOffset) {
            m_trajectory = makeTrajectory(goalOffsetSupplier.get(),
                    m_trajectory.sample(m_timer.get()).velocityMetersPerSecond);
            previousOffset = goalOffsetSupplier.get();
            m_timer.restart();
        }
        if (m_trajectory == null) {
            return;
        }
        double curTime = m_timer.get();
        var desiredState = m_trajectory.sample(curTime);

        this.desiredX = desiredState.poseMeters.getX();

        this.desiredY = desiredState.poseMeters.getY();

        // System.out.println("*****************"+goal);
        var targetChassisSpeeds = m_controller.calculate(m_swerve.getPose(), desiredState, goal.getRotation());
        var targetModuleStates = SwerveDriveSubsystem.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds);

        desiredXPublisher.set(desiredX);
        desiredYPublisher.set(desiredY);
        poseXPublisher.set(m_swerve.getPose().getX());
        poseYPublisher.set(m_swerve.getPose().getY());
        desiredRotPublisher.set(goal.getRotation().getRadians());

        rotSetpoint.set(m_rotationController.getSetpoint().position);


        poseRotPublisher.set(m_swerve.getPose().getRotation().getRadians());

        poseXErrorPublisher.set(xController.getPositionError());
        poseYErrorPublisher.set(yController.getPositionError());


        // holonomicXSetpoint.set(m_controller.getSetpoint());


        m_swerve.setModuleStates(targetModuleStates);

        // if( Math.abs(globalGoalTranslation.getX() - m_swerve.getPose().getX()) < 0.15
        // && Math.abs(globalGoalTranslation.getY() - m_swerve.getPose().getY()) < 0.15
        // ){
        // count++;
        // }

        // if(count >= 20){
        // isFinished = true;
        // }

        // if(count >= 60){
        //     isFinished = true;
        // }
    }

    // public double getDesiredX() {
    // // System.out.println("GLAAAAAAAAAAAAAAAAAA: "+ this.desiredX);
    // return this.desiredX;

    // }


    // @Override
    // public void initSendable(SendableBuilder builder) {
    //     super.initSendable(builder);
    //     builder.addDoubleProperty("X Setpoint", () -> xController.getSetpoint(), null);
    //     builder.addDoubleProperty("X Error", () -> xController.getPositionError(), null);
    //     builder.addDoubleProperty("X Measurment", () -> xController.getPositionError(), null);

    // }
    
}
