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
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
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
    DoublePublisher setpointUpper = inst.getTable("Arm Trajec").getDoubleTopic("Setpoint Upper").publish();
    DoublePublisher setpointLower = inst.getTable("Arm Trajec").getDoubleTopic("Setpoint Lower").publish();

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
            trajecConfig = new TrajectoryConfig(12, 5);
        } else {
            trajecConfig = new TrajectoryConfig(12, 2);
        }

        addRequirements(m_arm);
    }

    // Called when the command is initially scheduled.

    private Trajectory makeTrajectory() {

        // Cone
        InverseKinematicsAngle highGoalCone = new InverseKinematicsAngle(1.141496, 0.533380); // tuned for our mock up
        InverseKinematicsAngle midGoalCone = new InverseKinematicsAngle(1.465381, 0.018905); //not real
        InverseKinematicsAngle lowGoalCone = new InverseKinematicsAngle(2.21, 0); // not real
        InverseKinematicsAngle subCone = new InverseKinematicsAngle(1.440255, 0.006418); // tuned for our mock up

        // Cube
        InverseKinematicsAngle highGoalCube = new InverseKinematicsAngle(1.390242,0.648938); //not real
        InverseKinematicsAngle midGoalCube = new InverseKinematicsAngle(1.702710, 0.409309); //not real
        InverseKinematicsAngle lowGoalCube = new InverseKinematicsAngle(2.271662, -0.049849); //not real
        InverseKinematicsAngle subCube = new InverseKinematicsAngle(1.575104   , -0.064480);//not real
        InverseKinematicsAngle subToCube = new InverseKinematicsAngle(1.100452, -0.734924);//not real

        //Auton
        InverseKinematicsAngle autonLow = new InverseKinematicsAngle(2.277,0.8108); //not real
        InverseKinematicsAngle autonGrab = new InverseKinematicsAngle(2.394, 0.787); //not real
        // InverseKinematicsAngle lowGoalCube = new InverseKinematicsAngle(2.271662, -0.049849); //not real

        InverseKinematicsAngle safeBack = new InverseKinematicsAngle(1.97, -0.55);

        InverseKinematicsAngle safeGoalCone = new InverseKinematicsAngle(1.921083, -0.757354); //not real

    
        InverseKinematicsAngle safeGoalCube = new InverseKinematicsAngle(1.921083, -0.757354); //not real

        
        InverseKinematicsAngle safeWaypoint = m_arm.calculate(0.7, 0.44);

        // boolean notSafe = true;

        try{

        

        if(m_position == ArmPosition.SAFEBACK){
            return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(), Rotation2d.fromDegrees(-180)),
                    List.of(new Translation2d(safeWaypoint.upperTheta, safeWaypoint.lowerTheta)),
                    new Pose2d(safeBack.upperTheta, safeBack.lowerTheta, Rotation2d.fromDegrees(-180)),
                    trajecConfig);
        }

        if (!m_arm.cubeMode) {

            if (m_position == ArmPosition.SAFE) {
                return TrajectoryGenerator.generateTrajectory(
                        new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(), Rotation2d.fromDegrees(-180)),
                        List.of(new Translation2d(safeWaypoint.upperTheta, safeWaypoint.lowerTheta)),
                        new Pose2d(safeGoalCone.upperTheta, safeGoalCone.lowerTheta, Rotation2d.fromDegrees(-180)),
                        trajecConfig);
    
                // return null;
            }

            if (m_position == ArmPosition.HIGH) {
                return TrajectoryGenerator.generateTrajectory(
                        new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(), new Rotation2d(Math.PI / 2)),
                        List.of(),
                        new Pose2d(highGoalCone.upperTheta, highGoalCone.lowerTheta, new Rotation2d(Math.PI / 2)),
                        trajecConfig);

                // return TrajectoryGenerator.generateTrajectory(
                //         new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(), new Rotation2d(Math.PI / 2)),
                //         List.of(),
                //         new Pose2d(autonLow.upperTheta, autonLow.lowerTheta, new Rotation2d(Math.PI / 2)),
                //         trajecConfig);
            } else if (m_position == ArmPosition.MID) {
                return TrajectoryGenerator.generateTrajectory(
                        new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(), new Rotation2d(Math.PI / 2)),
                        List.of(),
                        new Pose2d(autonGrab.upperTheta, autonGrab.lowerTheta, new Rotation2d(Math.PI / 2)),
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
            } else if (m_position == ArmPosition.SUBTOCUBE) {
                return TrajectoryGenerator.generateTrajectory(
                        new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(), new Rotation2d(Math.PI / 2)),
                        List.of(),
                        new Pose2d(subToCube.upperTheta, subToCube.lowerTheta, new Rotation2d(Math.PI / 2)),
                        trajecConfig);
            }
        } else {

            if (m_position == ArmPosition.SAFE) {
                return TrajectoryGenerator.generateTrajectory(
                        new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(), Rotation2d.fromDegrees(-180)),
                        List.of(new Translation2d(safeWaypoint.upperTheta, safeWaypoint.lowerTheta)),
                        new Pose2d(safeGoalCube.upperTheta, safeGoalCube.lowerTheta, Rotation2d.fromDegrees(-180)),
                        trajecConfig);
    
                // return null;
            }

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
            } else if (m_position == ArmPosition.SUBTOCUBE) {
                return TrajectoryGenerator.generateTrajectory(
                        new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(), new Rotation2d(Math.PI / 2)),
                        List.of(),
                        new Pose2d(subToCube.upperTheta, subToCube.lowerTheta, new Rotation2d(Math.PI / 2)),
                        trajecConfig);
            }
        }

    }catch(TrajectoryGenerationException e){
        return null;

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
