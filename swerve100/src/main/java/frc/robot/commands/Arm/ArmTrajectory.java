package frc.robot.commands.Arm;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.ArmController;
import frc.robot.subsystems.Arm.ArmPosition;
// import frc.robot.subsystems.Arm.ArmTrajecs;
import frc.robot.subsystems.Arm.InverseKinematicsAngle;

public class ArmTrajectory extends CommandBase {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    DoublePublisher goalX = inst.getTable("Arm Trajec").getDoubleTopic("goalX").publish();
    DoublePublisher goalY = inst.getTable("Arm Trajec").getDoubleTopic("goalY").publish();
    DoublePublisher setpointX = inst.getTable("Arm Trajec").getDoubleTopic("setpointX").publish();
    DoublePublisher setpointY = inst.getTable("Arm Trajec").getDoubleTopic("setpointY").publish();
    DoublePublisher measurmentX = inst.getTable("Arm Trajec").getDoubleTopic("measurmentX").publish();
    DoublePublisher measurmentY = inst.getTable("Arm Trajec").getDoubleTopic("measurmentY").publish();

    private final Timer m_timer = new Timer();
    private final PIDController upperController;
    private final PIDController lowerController;

    private Trajectory m_trajectory;

    private final TrajectoryConfig trajecConfig;

    SimpleMotorFeedforward upperArmFeedforward = new SimpleMotorFeedforward(0.0, 0.3);
    SimpleMotorFeedforward lowerArmFeedforward = new SimpleMotorFeedforward(0.0, 0.3);

    // Pose2d m_goal;
    ArmController m_arm;
    private ArmPosition m_position;

    public ArmTrajectory(ArmPosition position, ArmController arm) {
        // Use addRequirements() here to declare subsystem dependencies.
        // m_goal = goal;
        m_arm = arm;

        m_position = position;

        // TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
        //         2, // velocity rad/s
        //         3 // accel rad/s^2
        // );

        upperController = new PIDController(3, 0, 0);
        lowerController = new PIDController(3, 0, 0);

        if (m_position != ArmPosition.SAFE) {
            trajecConfig = new TrajectoryConfig(12, 2);
        } else {
            trajecConfig = new TrajectoryConfig(12, 1);
        }

        addRequirements(m_arm);
    }

    // Called when the command is initially scheduled.

    private Trajectory makeTrajectory() {

        // Cone
        InverseKinematicsAngle highGoalCone = new InverseKinematicsAngle(1.05, 0.58); // tuned for our mock up
        InverseKinematicsAngle midGoalCone = new InverseKinematicsAngle(1.05, 0.58); //not real
        InverseKinematicsAngle lowGoalCone = new InverseKinematicsAngle(1.05, 0.58); // not real
        InverseKinematicsAngle subCone = new InverseKinematicsAngle(1.42, -0.50); // tuned for our mock up

        // Cube
        InverseKinematicsAngle highGoalCube = new InverseKinematicsAngle(2.02, 0); //not real
        InverseKinematicsAngle midGoalCube = new InverseKinematicsAngle(2.02, 0); //not real
        InverseKinematicsAngle lowGoalCube = new InverseKinematicsAngle(2.02, 0); //not real
        InverseKinematicsAngle subCube = new InverseKinematicsAngle(2.02, 0);//not real

        InverseKinematicsAngle safeGoal = m_arm.calculate(0.2, 0.125);

        InverseKinematicsAngle safeWaypoint = m_arm.calculate(0.7, 0.44);

        // boolean notSafe = true;

        if (m_position == ArmPosition.SAFE) {
            return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(), Rotation2d.fromDegrees(-180)),
                    List.of(new Translation2d(safeWaypoint.upperTheta, safeWaypoint.lowerTheta)),
                    new Pose2d(safeGoal.upperTheta, safeGoal.lowerTheta, Rotation2d.fromDegrees(-180)),
                    trajecConfig);
        }

        if (!m_arm.cubeMode) {
            if (m_position == ArmPosition.HIGH) {
                return TrajectoryGenerator.generateTrajectory(
                        new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(), new Rotation2d(Math.PI / 2)),
                        List.of(),
                        new Pose2d(highGoalCone.upperTheta, highGoalCone.lowerTheta, new Rotation2d(Math.PI / 2)),
                        trajecConfig);
            } else if (m_position == ArmPosition.MID) {
                return TrajectoryGenerator.generateTrajectory(
                        new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(), new Rotation2d(Math.PI / 2)),
                        List.of(),
                        new Pose2d(midGoalCone.upperTheta, midGoalCone.lowerTheta, new Rotation2d(Math.PI / 2)),
                        trajecConfig);
            } else if (m_position == ArmPosition.LOW) {
                return TrajectoryGenerator.generateTrajectory(
                        new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(), new Rotation2d(Math.PI / 2)),
                        List.of(),
                        new Pose2d(lowGoalCone.upperTheta, lowGoalCone.lowerTheta, new Rotation2d(Math.PI / 2)),
                        trajecConfig);
            } else if (m_position == ArmPosition.SUB) {
                return TrajectoryGenerator.generateTrajectory(
                        new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(), new Rotation2d(Math.PI / 2)),
                        List.of(),
                        new Pose2d(subCone.upperTheta, subCone.lowerTheta, new Rotation2d(Math.PI / 2)),
                        trajecConfig);
            }
        } else {
            if (m_position == ArmPosition.HIGH) {
                return TrajectoryGenerator.generateTrajectory(
                        new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(), new Rotation2d(Math.PI / 2)),
                        List.of(),
                        new Pose2d(highGoalCube.upperTheta, highGoalCube.lowerTheta, new Rotation2d(Math.PI / 2)),
                        trajecConfig);
            } else if (m_position == ArmPosition.MID) {
                return TrajectoryGenerator.generateTrajectory(
                        new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(), new Rotation2d(Math.PI / 2)),
                        List.of(),
                        new Pose2d(midGoalCube.upperTheta, midGoalCube.lowerTheta, new Rotation2d(Math.PI / 2)),
                        trajecConfig);
            } else if (m_position == ArmPosition.LOW) {
                return TrajectoryGenerator.generateTrajectory(
                        new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(), new Rotation2d(Math.PI / 2)),
                        List.of(),
                        new Pose2d(lowGoalCube.upperTheta, lowGoalCube.lowerTheta, new Rotation2d(Math.PI / 2)),
                        trajecConfig);
            } else if (m_position == ArmPosition.SUB) {
                return TrajectoryGenerator.generateTrajectory(
                        new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(), new Rotation2d(Math.PI / 2)),
                        List.of(),
                        new Pose2d(subCube.upperTheta, subCube.lowerTheta, new Rotation2d(Math.PI / 2)),
                        trajecConfig);
            }
        }

        return null;

        // if(m_position == ArmPosition.SAFE){
        // return TrajectoryGenerator.generateTrajectory(
        // new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(),
        // Rotation2d.fromDegrees(-180)),
        // List.of(),
        // new Pose2d(safeGoal.upperTheta, safeGoal.lowerTheta,
        // Rotation2d.fromDegrees(-180)),
        // trajecConfig);
        // } else {
        // return TrajectoryGenerator.generateTrajectory(
        // new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(), new
        // Rotation2d(Math.PI/2)),
        // List.of(),
        // new Pose2d(highGoalCone.upperTheta, highGoalCone.lowerTheta, new
        // Rotation2d(Math.PI/2)),
        // trajecConfig);
        // }

        // return ArmTrajecs.getTrajectory(() -> m_arm.getUpperArm(), ()
        // ->m_arm.getLowerArm(), m_position, m_arm, trajecConfig);

    }

    @Override
    public void initialize() {
        m_timer.restart();
        m_trajectory = makeTrajectory();
    }

    // Called every time the scheduler runs while the command is scheduled.
    public void execute() {
        double curTime = m_timer.get();
        var desiredState = m_trajectory.sample(curTime);

        double desiredUpper = desiredState.poseMeters.getX();
        double desiredLower = desiredState.poseMeters.getY();

        double upperSpeed = upperController.calculate(m_arm.getUpperArm(), desiredUpper);
        double lowerSpeed = lowerController.calculate(m_arm.getLowerArm(), desiredLower);

        // double uFF =
        // upperArmFeedforward.calculate(desiredState.velocityMetersPerSecond, 0);
        // double lFF =
        // upperArmFeedforward.calculate(desiredState.velocityMetersPerSecond, 0);

        m_arm.setUpperArm(upperSpeed);
        m_arm.setLowerArm(lowerSpeed);

        measurmentX.set(m_arm.getUpperArm());
        measurmentY.set(m_arm.getLowerArm());
        setpointX.set(desiredUpper);
        setpointY.set(desiredLower);

    }

    @Override
    public void end(boolean interrupted) {
        m_arm.setUpperArm(0);
        m_arm.setLowerArm(0);
    }
  
}
