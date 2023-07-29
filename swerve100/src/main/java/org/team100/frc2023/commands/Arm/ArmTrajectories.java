package org.team100.frc2023.commands.Arm;

import java.util.List;

import org.team100.frc2023.subsystems.arm.ArmController;
import org.team100.frc2023.subsystems.arm.ArmPosition;
import org.team100.lib.subsystems.arm.ArmAngles;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;

public class ArmTrajectories {
    // Cone
    private static final ArmAngles highGoalCone = new ArmAngles(0.494, 1.178);
    private static final ArmAngles midGoalCone = new ArmAngles(0.138339, 1.609977);
    private static final ArmAngles lowGoalCone = new ArmAngles(0, 2.21);
    private static final ArmAngles subCone = new ArmAngles(-0.338940, ArmController.coneSubVal);

    // Cube
    private static final ArmAngles highGoalCube = new ArmAngles(0.316365, 1.147321);
    private static final ArmAngles midGoalCube = new ArmAngles(0.089803, 1.681915);
    private static final ArmAngles lowGoalCube = new ArmAngles(-0.049849, 2.271662);
    private static final ArmAngles subCube = new ArmAngles(-0.341841, 1.361939);
    private static final ArmAngles subToCube = new ArmAngles(-0.341841, 1.361939);

    private static final ArmAngles safeBack = new ArmAngles(-0.55, 1.97);
    private static final ArmAngles safeGoalCone = new ArmAngles(-0.639248, 1.838205);
    private static final ArmAngles safeGoalCube = new ArmAngles(-0.639248, 1.838205);
    private static final ArmAngles safeWaypoint = new ArmAngles(-0.394089, 1.226285);

    private final TrajectoryConfig trajecConfig;

    public ArmTrajectories(TrajectoryConfig config) {
        trajecConfig = config;
    }

    public Trajectory makeTrajectory(ArmAngles start, ArmPosition goal, boolean cubeMode) {
        if (start == null) // unreachable 
            return null;
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
        return withList(start, List.of(new Translation2d(mid.th2, mid.th1)), end, degrees);
    }

    private Trajectory withList(ArmAngles start, List<Translation2d> list, ArmAngles end, double degrees) {
        try {
            return TrajectoryGenerator.generateTrajectory(startPose(start, degrees), list, endPose(end, degrees),
                    trajecConfig);
        } catch (TrajectoryGenerationException e) {
            e.printStackTrace();
            return null;
        }
    }

    private Pose2d startPose(ArmAngles start, double degrees) {
        return new Pose2d(start.th2, start.th1, Rotation2d.fromDegrees(degrees));
    }

    private Pose2d endPose(ArmAngles angles, double degrees) {
        return new Pose2d(angles.th2, angles.th1, Rotation2d.fromDegrees(degrees));
    }

}
