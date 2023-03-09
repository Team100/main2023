package frc.robot.subsystems.Arm;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

/** Add your docs here. */
public class ArmTrajecs {

    public static Trajectory getTrajectory(Supplier<Double> getUpperArm, Supplier<Double> getLowerArm,
            ArmPosition position, ArmController m_arm, TrajectoryConfig trajecConfig) {

        if (position == ArmPosition.HIGH) {
            return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(getUpperArm.get(), getLowerArm.get(), new Rotation2d(Math.PI / 2)),
                    List.of(),
                    new Pose2d(1.05, 0.58, new Rotation2d(Math.PI / 2)),
                    trajecConfig);
        } else {
            return TrajectoryGenerator.generateTrajectory(
                    new Pose2d(getUpperArm.get(), getLowerArm.get(), Rotation2d.fromDegrees(-180)),
                    List.of(),
                    new Pose2d(0.2, 0.125, Rotation2d.fromDegrees(-180)),
                    trajecConfig);

        }
    }

    public static Trajectory getConeLowTraj(ArmController arm, TrajectoryConfig trajecConfig) {
        // InverseKinematicsAngle waypoint = new InverseKinematicsAngle();
        InverseKinematicsAngle goal = new InverseKinematicsAngle();

        return TrajectoryGenerator.generateTrajectory(
                new Pose2d(arm.getUpperArm(), arm.getLowerArm(), new Rotation2d(Math.PI / 2)),
                List.of(),
                new Pose2d(goal.upperTheta, goal.lowerTheta, new Rotation2d(Math.PI / 2)),
                trajecConfig);
    }

    public static Trajectory getConeMidTraj(ArmController arm, TrajectoryConfig trajecConfig) {
        // InverseKinematicsAngle waypoint = new InverseKinematicsAngle();
        InverseKinematicsAngle goal = new InverseKinematicsAngle();

        return TrajectoryGenerator.generateTrajectory(
                new Pose2d(arm.getUpperArm(), arm.getLowerArm(), new Rotation2d(Math.PI / 2)),
                List.of(),
                new Pose2d(goal.upperTheta, goal.lowerTheta, new Rotation2d(Math.PI / 2)),
                trajecConfig);
    }

    public static Trajectory getConeHighTraj(ArmController arm, TrajectoryConfig trajecConfig) {
        // InverseKinematicsAngle waypoint = new InverseKinematicsAngle();
        InverseKinematicsAngle goal = new InverseKinematicsAngle(1.05, 0.58);

        return TrajectoryGenerator.generateTrajectory(
                new Pose2d(arm.getUpperArm(), arm.getLowerArm(), new Rotation2d(Math.PI / 2)),
                List.of(),
                new Pose2d(goal.upperTheta, goal.lowerTheta, new Rotation2d(Math.PI / 2)),
                trajecConfig);
    }

    public static Trajectory getCubeLowTraj(ArmController arm, TrajectoryConfig trajecConfig) {
        // InverseKinematicsAngle waypoint = new InverseKinematicsAngle();
        InverseKinematicsAngle goal = new InverseKinematicsAngle();

        return TrajectoryGenerator.generateTrajectory(
                new Pose2d(arm.getUpperArm(), arm.getLowerArm(), new Rotation2d(Math.PI / 2)),
                List.of(),
                new Pose2d(goal.upperTheta, goal.lowerTheta, new Rotation2d(Math.PI / 2)),
                trajecConfig);
    }

    public static Trajectory getCubeMidTraj(ArmController arm, TrajectoryConfig trajecConfig) {
        // InverseKinematicsAngle waypoint = new InverseKinematicsAngle();
        InverseKinematicsAngle goal = new InverseKinematicsAngle();

        return TrajectoryGenerator.generateTrajectory(
                new Pose2d(arm.getUpperArm(), arm.getLowerArm(), new Rotation2d(Math.PI / 2)),
                List.of(),
                new Pose2d(goal.upperTheta, goal.lowerTheta, new Rotation2d(Math.PI / 2)),
                trajecConfig);
    }

    public static Trajectory getCubeHighTraj(ArmController arm, TrajectoryConfig trajecConfig) {
        // InverseKinematicsAngle waypoint = new InverseKinematicsAngle();
        InverseKinematicsAngle goal = new InverseKinematicsAngle();

        return TrajectoryGenerator.generateTrajectory(
                new Pose2d(arm.getUpperArm(), arm.getLowerArm(), new Rotation2d(Math.PI / 2)),
                List.of(),
                new Pose2d(goal.upperTheta, goal.lowerTheta, new Rotation2d(Math.PI / 2)),
                trajecConfig);
    }

    public static Trajectory getCubeSubstation(ArmController arm, TrajectoryConfig trajecConfig) {
        // InverseKinematicsAngle waypoint = new InverseKinematicsAngle();
        InverseKinematicsAngle goal = new InverseKinematicsAngle();

        return TrajectoryGenerator.generateTrajectory(
                new Pose2d(arm.getUpperArm(), arm.getLowerArm(), new Rotation2d(Math.PI / 2)),
                List.of(),
                new Pose2d(goal.upperTheta, goal.lowerTheta, new Rotation2d(Math.PI / 2)),
                trajecConfig);
    }

    public static Trajectory getConeSubstation(ArmController arm, TrajectoryConfig trajecConfig) {
        // InverseKinematicsAngle waypoint = new InverseKinematicsAngle();
        InverseKinematicsAngle goal = new InverseKinematicsAngle();

        return TrajectoryGenerator.generateTrajectory(
                new Pose2d(arm.getUpperArm(), arm.getLowerArm(), new Rotation2d(Math.PI / 2)),
                List.of(),
                new Pose2d(goal.upperTheta, goal.lowerTheta, new Rotation2d(Math.PI / 2)),
                trajecConfig);
    }

    public static Trajectory getSafePosition(ArmController arm, TrajectoryConfig trajecConfig) {
        // InverseKinematicsAngle waypoint = new InverseKinematicsAngle(0.7, 0.44);
        InverseKinematicsAngle goal = new InverseKinematicsAngle(0.2, 0.125);

        return TrajectoryGenerator.generateTrajectory(
                new Pose2d(arm.getUpperArm(), arm.getLowerArm(), Rotation2d.fromDegrees(-180)),
                List.of(),
                new Pose2d(goal.upperTheta, goal.lowerTheta, Rotation2d.fromDegrees(-180)),
                trajecConfig);
    }

}
