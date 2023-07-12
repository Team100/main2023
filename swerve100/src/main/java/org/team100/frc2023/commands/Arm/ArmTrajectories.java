package org.team100.frc2023.commands.Arm;

import java.util.List;

import org.team100.frc2023.subsystems.arm.ArmController;
import org.team100.frc2023.subsystems.arm.ArmPosition;
import org.team100.frc2023.subsystems.arm.ArmAngles;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;

public class ArmTrajectories {
    // Cone
    private static final ArmAngles highGoalCone = new ArmAngles(1.178, 0.494);
    private static final ArmAngles midGoalCone = new ArmAngles(1.609977, 0.138339);
    private static final ArmAngles lowGoalCone = new ArmAngles(2.21, 0);
    private static final ArmAngles subCone = new ArmAngles(ArmController.coneSubVal, -0.338940);

    // Cube
    private static final ArmAngles highGoalCube = new ArmAngles(1.147321, 0.316365);
    private static final ArmAngles midGoalCube = new ArmAngles(1.681915, 0.089803);
    private static final ArmAngles lowGoalCube = new ArmAngles(2.271662, -0.049849);
    private static final ArmAngles subCube = new ArmAngles(1.361939, -0.341841);
    private static final ArmAngles subToCube = new ArmAngles(1.361939, -0.341841);

    private static final ArmAngles safeBack = new ArmAngles(1.97, -0.55);
    private static final ArmAngles safeGoalCone = new ArmAngles(1.838205, -0.639248);
    private static final ArmAngles safeGoalCube = new ArmAngles(1.838205, -0.639248);
    private static final ArmAngles safeWaypoint = new ArmAngles(1.226285, -0.394089);

    private final TrajectoryConfig trajecConfig;

    public ArmTrajectories(TrajectoryConfig config) {
        trajecConfig = config;
    }

    public Trajectory makeTrajectory(ArmAngles start, ArmPosition goal, boolean cubeMode) {
        switch (goal) {
            case SAFEBACK:
                return twoPoint(start, safeWaypoint, safeBack, -180);
            case SAFE:
                if (cubeMode)
                    return twoPoint(start, safeWaypoint, safeGoalCube, -180);
                return twoPoint(start, safeWaypoint, safeGoalCone, -180);
            case SAFEWAYPOINT:
                return onePoint(start, safeWaypoint, -180);
            case HIGH:
                if (cubeMode)
                    return onePoint(start, highGoalCube, 90);
                return onePoint(start, highGoalCone, 90);
            case MID:
                if (cubeMode)
                    return onePoint(start, midGoalCube, 90);
                return onePoint(start, midGoalCone, 90);
            case LOW:
                if (cubeMode)
                    return onePoint(start, lowGoalCube, 90);
                return onePoint(start, lowGoalCone, 90);
            case SUB:
                if (cubeMode)
                    return onePoint(start, subCube, 90);
                return onePoint(start, subCone, 90);
            case SUBTOCUBE:
                if (cubeMode)
                    return onePoint(start, subToCube, 90);
                return onePoint(start, subToCube, 90);
        }

        return null;
    }

    /** from current location to an endpoint */
    private Trajectory onePoint(ArmAngles start, ArmAngles end, double degrees) {
        return withList(start, List.of(), end, degrees);
    }

    /** from current location, through a waypoint, to an endpoint */
    private Trajectory twoPoint(ArmAngles start, ArmAngles mid, ArmAngles end, double degrees) {
        return withList(start, List.of(new Translation2d(mid.upperTheta, mid.lowerTheta)), end, degrees);
    }

    private Trajectory withList(ArmAngles start, List<Translation2d> list, ArmAngles end, double degrees) {
        try {
            return TrajectoryGenerator.generateTrajectory(startPose(start, degrees), list, endPose(end, degrees),
                    trajecConfig);
        } catch (TrajectoryGenerationException e) {
            return null;
        }
    }

    private Pose2d startPose(ArmAngles start, double degrees) {
        return new Pose2d(start.upperTheta, start.lowerTheta, Rotation2d.fromDegrees(degrees));
    }

    private Pose2d endPose(ArmAngles angles, double degrees) {
        return new Pose2d(angles.upperTheta, angles.lowerTheta, Rotation2d.fromDegrees(degrees));
    }

}
