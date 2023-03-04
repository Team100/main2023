package frc.robot;

import java.io.IOException;
import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;

public class malformedSplineTest {
    @Test
    public void malformedSplineTest() throws IOException {
        try {
            Trajectory malformed = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d()),
            List.of(),
            new Pose2d(0, 0, new Rotation2d()),
            new TrajectoryConfig(6, 3)
            );
        } catch(TrajectoryGenerationException e) {
            
        }
    }
}
