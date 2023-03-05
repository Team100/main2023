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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.ArmController;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Arm.InverseKinematicsAngle;

public class ArmTrajectory extends CommandBase {

  
  /** Creates a new ArmTrajectory. */

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

  private double xPos;
  private double yPos;
  private Trajectory m_trajectory;

  private final TrajectoryConfig trajecConfig;


  SimpleMotorFeedforward upperArmFeedforward = new SimpleMotorFeedforward(0.0, 0.3);
  SimpleMotorFeedforward lowerArmFeedforward = new SimpleMotorFeedforward(0.0, 0.3);




  // Pose2d m_goal;
  ArmController m_arm;
  private ArmPosition m_position;
  public ArmTrajectory(double x, double y, ArmPosition position, ArmController arm) {
    // Use addRequirements() here to declare subsystem dependencies.

    xPos = x;
    yPos = y;
    // m_goal = goal;
    m_arm = arm;

    m_position = position;

    

    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
      2, // velocity rad/s
      3 // accel rad/s^2
    );


    upperController = new PIDController(3, 0, 0);
    lowerController = new PIDController(3, 0, 0);


    if(m_position != ArmPosition.inward){
        trajecConfig = new TrajectoryConfig(12, 2);
    }else{
        trajecConfig = new TrajectoryConfig(12, 1);
    }

    addRequirements(m_arm);
  }



  // Called when the command is initially scheduled.

  private Trajectory makeTrajectory() {

    InverseKinematicsAngle goal = m_arm.calculate(xPos, yPos);

    // InverseKinematicsAngle highGoal = m_arm.calculate(1.24, 1.25);

    InverseKinematicsAngle highGoal = new InverseKinematicsAngle(0.95, 0.59 );

    InverseKinematicsAngle midGoal = m_arm.calculate(1.24, 1.25);
    InverseKinematicsAngle lowGoal = m_arm.calculate(1.24, 1.25);

    InverseKinematicsAngle inWaypoint = m_arm.calculate(0.7, 0.44);


    if(m_position == ArmPosition.high){
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(), new Rotation2d(Math.PI/2)),
            List.of(),
            new Pose2d(highGoal.upperTheta, highGoal.lowerTheta, new Rotation2d(Math.PI/2)),
            trajecConfig);
    } else if(m_position == ArmPosition.mid){
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(), new Rotation2d(Math.PI/2)),
            List.of(),
            new Pose2d(midGoal.upperTheta, midGoal.lowerTheta, new Rotation2d(Math.PI/2)),
            trajecConfig);
    } else if(m_position == ArmPosition.low){
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(), new Rotation2d(Math.PI/2)),
            List.of(),
            new Pose2d(lowGoal.upperTheta, lowGoal.lowerTheta, new Rotation2d(Math.PI/2)),
            trajecConfig);
    }else if(m_position == ArmPosition.inward){
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(), Rotation2d.fromDegrees(-180)),
            List.of(new Translation2d(inWaypoint.upperTheta, inWaypoint.lowerTheta)),
            new Pose2d(goal.upperTheta, goal.lowerTheta, Rotation2d.fromDegrees(-180)),
            trajecConfig);
    }

    return null;
    
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

    double uFF = upperArmFeedforward.calculate(desiredState.velocityMetersPerSecond, 0);
    double lFF = upperArmFeedforward.calculate(desiredState.velocityMetersPerSecond, 0);


    m_arm.setUpperArm(upperSpeed);
    m_arm.setLowerArm(lowerSpeed);

    measurmentX.set(m_arm.getUpperArm());
    measurmentY.set(m_arm.getLowerArm());
    setpointX.set(desiredUpper);
    setpointY.set(desiredLower);


    
    
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
