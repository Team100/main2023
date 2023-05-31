package team100.frc2023.autonomous;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import team100.frc2023.subsystems.AHRSClass;
import team100.frc2023.subsystems.SwerveDriveSubsystem;

/** Add your docs here. */
public class MoveToAprilTag extends TrajectoryCommand {

    public MoveToAprilTag(SwerveDriveSubsystem m_robotDrive, Trajectory trajectory, AHRSClass gyro) {
        super(m_robotDrive, trajectory, gyro);
    }

    public static MoveToAprilTag newMoveToAprilTag(SwerveDriveSubsystem m_robotDrive, Supplier<Pose2d> getPose, int tagID, AHRSClass gyro) {
        // HashTag hashTag = new HashTag();
        Pose2d aprilPose = m_robotDrive.visionDataProvider.layout.getTagPose(tagID).get().toPose2d();
        
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                2,
                1)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(SwerveDriveSubsystem.kDriveKinematics);

        System.out.println("MOVE TO APRIL TAG****************************************************************");


        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                getPose.get(),
                List.of(),
                new Pose2d(aprilPose.getX()-1, aprilPose.getY(), new Rotation2d(aprilPose.getRotation().getDegrees())),
                trajectoryConfig);

        return new MoveToAprilTag(m_robotDrive, exampleTrajectory, gyro);

    }


}
