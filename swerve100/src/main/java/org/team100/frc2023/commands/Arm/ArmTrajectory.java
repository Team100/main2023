package org.team100.frc2023.commands.Arm;

import java.util.List;

import org.team100.frc2023.subsystems.arm.ArmController;
import org.team100.frc2023.subsystems.arm.ArmPosition;
import org.team100.frc2023.subsystems.arm.InverseKinematicsAngle;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmTrajectory extends CommandBase {

    // Cone
    private static final InverseKinematicsAngle highGoalCone = new InverseKinematicsAngle(1.178, 0.494);
    private static final InverseKinematicsAngle midGoalCone = new InverseKinematicsAngle(1.609977, 0.138339);
    private static final InverseKinematicsAngle lowGoalCone = new InverseKinematicsAngle(2.21, 0);
    private static final InverseKinematicsAngle subCone = new InverseKinematicsAngle(ArmController.coneSubVal,
            -0.338940);

    // Cube
    private static final InverseKinematicsAngle highGoalCube = new InverseKinematicsAngle(1.147321, 0.316365);
    private static final InverseKinematicsAngle midGoalCube = new InverseKinematicsAngle(1.681915, 0.089803);
    private static final InverseKinematicsAngle lowGoalCube = new InverseKinematicsAngle(2.271662, -0.049849);
    private static final InverseKinematicsAngle subCube = new InverseKinematicsAngle(1.361939, -0.341841);
    private static final InverseKinematicsAngle subToCube = new InverseKinematicsAngle(1.361939, -0.341841);

    private static final InverseKinematicsAngle safeBack = new InverseKinematicsAngle(1.97, -0.55);
    private static final InverseKinematicsAngle safeGoalCone = new InverseKinematicsAngle(1.838205, -0.639248);
    private static final InverseKinematicsAngle safeGoalCube = new InverseKinematicsAngle(1.838205, -0.639248);
    private static final InverseKinematicsAngle safeWaypoint = new InverseKinematicsAngle(1.226285, -0.394089);

    private final ArmController m_arm;
    private final ArmPosition m_position;
    private final PIDController upperController;
    private final PIDController lowerController;
    private final PIDController upperDownController;
    private final PIDController lowerDownController;
    private final TrajectoryConfig trajecConfig;

    private final NetworkTableInstance inst;
    private final DoublePublisher measurmentX;
    private final DoublePublisher measurmentY;
    private final DoublePublisher setpointUpper;
    private final DoublePublisher setpointLower;

    private final Timer m_timer;

    private Trajectory m_trajectory;

    private boolean isSafeWaypoint = false;

    public ArmTrajectory(ArmPosition position, ArmController arm) {
        m_arm = arm;
        m_position = position;
        m_timer = new Timer();
        upperController = new PIDController(4, 0.2, 0.05);
        upperController.setTolerance(0.001);
        lowerController = new PIDController(3, 0, 0.1);
        lowerController.setTolerance(0.001);
        upperDownController = new PIDController(2.5, 0, 0);
        lowerDownController = new PIDController(2.5, 0, 0);

        if (m_position != ArmPosition.SAFE) {
            trajecConfig = new TrajectoryConfig(12, 2);
        } else {
            trajecConfig = new TrajectoryConfig(9, 1.5);
        }

        inst = NetworkTableInstance.getDefault();
        measurmentX = inst.getTable("Arm Trajec").getDoubleTopic("measurmentX").publish();
        measurmentY = inst.getTable("Arm Trajec").getDoubleTopic("measurmentY").publish();
        setpointUpper = inst.getTable("Arm Trajec").getDoubleTopic("Setpoint Upper").publish();
        setpointLower = inst.getTable("Arm Trajec").getDoubleTopic("Setpoint Lower").publish();

        addRequirements(m_arm);
    }

    private Pose2d startPose(double degrees) {
        return new Pose2d(m_arm.getUpperArm(), m_arm.getLowerArm(), Rotation2d.fromDegrees(degrees));
    }

    private Pose2d endPose(InverseKinematicsAngle angles, double degrees) {
        return new Pose2d(angles.upperTheta, angles.lowerTheta, Rotation2d.fromDegrees(degrees));
    }

    /** from current location to an endpoint */
    private Trajectory onePoint(InverseKinematicsAngle end, double degrees) {
        return withList(List.of(), end, degrees);
    }

    /** from current location, through a waypoint, to an endpoint */
    private Trajectory twoPoint(InverseKinematicsAngle mid, InverseKinematicsAngle end, double degrees) {
        return withList(List.of(new Translation2d(mid.upperTheta, mid.lowerTheta)), end, degrees);
    }

    private Trajectory withList(List<Translation2d> list, InverseKinematicsAngle end, double degrees) {
        try {
            return TrajectoryGenerator.generateTrajectory(startPose(degrees), list, endPose(end, degrees),
                    trajecConfig);
        } catch (TrajectoryGenerationException e) {
            return null;
        }
    }

    private Trajectory makeTrajectory() {
        isSafeWaypoint = false;
        switch (m_position) {
            case SAFEBACK:
                return twoPoint(safeWaypoint, safeBack, -180);
            case SAFE:
                if (m_arm.cubeMode)
                    return twoPoint(safeWaypoint, safeGoalCube, -180);
                return twoPoint(safeWaypoint, safeGoalCone, -180);
            case SAFEWAYPOINT:
                isSafeWaypoint = true;
                return onePoint(safeWaypoint, -180);
            case HIGH:
                if (m_arm.cubeMode)
                    return onePoint(highGoalCube, 90);
                return onePoint(highGoalCone, 90);
            case MID:
                if (m_arm.cubeMode)
                    return onePoint(midGoalCube, 90);
                return onePoint(midGoalCone, 90);
            case LOW:
                if (m_arm.cubeMode)
                    return onePoint(lowGoalCube, 90);
                return onePoint(lowGoalCone, 90);
            case SUB:
                if (m_arm.cubeMode)
                    return onePoint(subCube, 90);
                return onePoint(subCone, 90);
            case SUBTOCUBE:
                if (m_arm.cubeMode)
                    return onePoint(subToCube, 90);
                return onePoint(subToCube, 90);
        }

        return null;
    }

    @Override
    public void initialize() {
        m_timer.restart();
        m_trajectory = makeTrajectory();
    }

    public void execute() {

        if (m_trajectory == null) {
            return;
        }
        double curTime = m_timer.get();

        State desiredState = m_trajectory.sample(curTime);

        double desiredUpper = desiredState.poseMeters.getX();
        double desiredLower = desiredState.poseMeters.getY();

        double upperSpeed = 0;
        double lowerSpeed = 0;

        double lowerFeed = 0;

        if (m_position == ArmPosition.SAFE) {
            upperSpeed = upperDownController.calculate(m_arm.getUpperArm(), desiredUpper);
            lowerSpeed = lowerDownController.calculate(m_arm.getLowerArm(), desiredLower);

        } else {
            upperSpeed = upperController.calculate(m_arm.getUpperArm(), desiredUpper);
            lowerSpeed = lowerController.calculate(m_arm.getLowerArm(), desiredLower);

            // lowerFeed = 0.01 * Math.signum(lowerController.getPositionError());
        }

        m_arm.setUpperArm(upperSpeed);
        m_arm.setLowerArm(lowerSpeed + lowerFeed);

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
        if (isSafeWaypoint) {
            return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
        } else {
            return false;
        }
    }
}
