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

    private final PIDController upperDownController;
    private final PIDController lowerDownController;

    private Trajectory m_trajectory;

    private final TrajectoryConfig trajecConfig;

    SimpleMotorFeedforward upperArmFeedforward = new SimpleMotorFeedforward(0.0, 0.3);
    SimpleMotorFeedforward lowerArmFeedforward = new SimpleMotorFeedforward(0.0, 0.3);

    // Pose2d m_goal;
    ArmController m_arm;
    private ArmPosition m_position;

    private boolean isSafeWaypoint = false;

    public ArmTrajectory(ArmPosition position, ArmController arm) {
        // Use addRequirements() here to declare subsystem dependencies.
        // m_goal = goal;
        m_arm = arm;

        m_position = position;

        // TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
        //         2, // velocity rad/s
        //         3 // accel rad/s^2
        // );

        upperController = new PIDController(4, 0.2, 0.05);
        lowerController = new PIDController(3, 0, 0.1);

        upperDownController = new PIDController(2.5, 0, 0);
        lowerDownController = new PIDController(2.5, 0, 0);

        upperController.setTolerance(0.001);
        lowerController.setTolerance(0.001);

        if (m_position != ArmPosition.SAFE) {
            trajecConfig = new TrajectoryConfig(12, 2  );
        } else {
            trajecConfig = new TrajectoryConfig(9, 1.5);
        }

        addRequirements(m_arm);
    }

    // Called when the command is initially scheduled.

    private Trajectory makeTrajectory() {

        // Cone
        InverseKinematicsAngle highGoalCone = new InverseKinematicsAngle(1.141496, 0.533380); // tuned for our mock up
        InverseKinematicsAngle midGoalCone = new InverseKinematicsAngle(1.545253, 0.275188); //not real
        InverseKinematicsAngle lowGoalCone = new InverseKinematicsAngle(2.21, 0); // not real
        InverseKinematicsAngle subCone = new InverseKinematicsAngle(1.408241, -0.006084); // tuned for our mock up

        // Cube
        InverseKinematicsAngle highGoalCube = new InverseKinematicsAngle(1.206391,0.526467); //not real
        InverseKinematicsAngle midGoalCube = new InverseKinematicsAngle(1.687309, 0.150418); //not real
        InverseKinematicsAngle lowGoalCube = new InverseKinematicsAngle(2.271662, -0.049849); //not real
        InverseKinematicsAngle subCube = new InverseKinematicsAngle(1.480104   , -0.04);//not real
        InverseKinematicsAngle subToCube = new InverseKinematicsAngle(1.100452, -0.734924);//not real

        //Auton
        InverseKinematicsAngle autonLow = new InverseKinematicsAngle(2.277,0.8108); //not real
        InverseKinematicsAngle autonGrab = new InverseKinematicsAngle(2.394, 0.787); //not real
        // InverseKinematicsAngle lowGoalCube = new InverseKinematicsAngle(2.271662, -0.049849); //not real

        InverseKinematicsAngle safeBack = new InverseKinematicsAngle(1.97, -0.55);

        InverseKinematicsAngle safeGoalCone = new InverseKinematicsAngle(1.838205, -0.639248); //not real

    
        InverseKinematicsAngle safeGoalCube = new InverseKinematicsAngle(1.680436, -0.298320); //not real

        
        InverseKinematicsAngle safeWaypoint = new InverseKinematicsAngle(1.226285, -0.394089); //not real



        InverseKinematicsAngle subSafeWaypoint = m_arm.calculate(1.329387, -0.238770);

        // boolean notSafe = true;

        isSafeWaypoint = false;

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

            if(m_position == ArmPosition.SAFEWAYPOINT){
                isSafeWaypoint = true;

                return TrajectoryGenerator.generateTrajectory(
                        new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(), Rotation2d.fromDegrees(-180)),
                        List.of(),
                        new Pose2d(safeWaypoint.upperTheta, safeWaypoint.lowerTheta, Rotation2d.fromDegrees(-180)),
                        trajecConfig);

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

            if(m_position == ArmPosition.SAFEWAYPOINT){

                isSafeWaypoint = true;
                return TrajectoryGenerator.generateTrajectory(
                        new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(), Rotation2d.fromDegrees(-180)),
                        List.of(),
                        new Pose2d(safeWaypoint.upperTheta, safeWaypoint.lowerTheta, Rotation2d.fromDegrees(-180)),
                        trajecConfig);
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

        double upperSpeed = 0;
        double lowerSpeed = 0;

        if(m_position == ArmPosition.SAFE){
            upperSpeed = upperDownController.calculate(m_arm.getUpperArm(), desiredUpper);
            lowerSpeed = lowerDownController.calculate(m_arm.getLowerArm(), desiredLower);    
        } else {
            upperSpeed = upperController.calculate(m_arm.getUpperArm(), desiredUpper);
            lowerSpeed = lowerController.calculate(m_arm.getLowerArm(), desiredLower);
        }

        
        m_arm.setUpperArm(upperSpeed);
        m_arm.setLowerArm(lowerSpeed);

        measurmentX.set(m_arm.getUpperArm());
        measurmentY.set(m_arm.getLowerArm());

        setpointUpper.set(desiredUpper);
        setpointLower.set(desiredLower);

    }

    @Override
    public void end(boolean interrupted) {
        m_arm.setUpperArm(0);
        m_arm.setLowerArm(0);
    }

    @Override
    public boolean isFinished() {

        if(isSafeWaypoint){
            return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
        } else {
            return false;
        }
        
    }


  
}
