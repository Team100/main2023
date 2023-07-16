package com.team254.frc2022.paths;
import org.junit.jupiter.api.Test;

/**
 * Ensure all trajectories are valid
 */
public class TrajectoryGeneratorTest {
    @Test
    public void testGenerateTrajectories() {
        TrajectoryGenerator.getInstance().generateTrajectories();
    }
}