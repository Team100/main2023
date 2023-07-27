package org.team100.lib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team254.lib.swerve.SwerveDriveKinematics;
import com.team254.lib.swerve.SwerveModuleState;
import com.team254.lib.swerve.SwerveSetpoint;
import com.team254.lib.swerve.SwerveSetpointGenerator;
import com.team254.lib.util.Util;

public class SwerveSetpointGeneratorTest2 {

    protected final static double kRobotSide = 0.616; // m
    protected final static SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(kRobotSide / 2.0, kRobotSide / 2.0),
            // Front right
            new Translation2d(kRobotSide / 2.0, -kRobotSide / 2.0),
            // Back left
            new Translation2d(-kRobotSide / 2.0, kRobotSide / 2.0),
            // Back right
            new Translation2d(-kRobotSide / 2.0, -kRobotSide / 2.0));
    protected static SwerveSetpointGenerator.KinematicLimits kKinematicLimits = new SwerveSetpointGenerator.KinematicLimits();
    protected static SwerveSetpointGenerator2.KinematicLimits kKinematicLimits2 = new SwerveSetpointGenerator2.KinematicLimits();
    static {
        kKinematicLimits.kMaxDriveVelocity = 5.0; // m/s
        kKinematicLimits.kMaxDriveAcceleration = 10.0; // m/s^2
        kKinematicLimits.kMaxSteeringVelocity = Math.toRadians(1500.0); // rad/s
    };
    static {
        kKinematicLimits2.kMaxDriveVelocity = 5.0; // m/s
        kKinematicLimits2.kMaxDriveAcceleration = 10.0; // m/s^2
        kKinematicLimits2.kMaxSteeringVelocity = Math.toRadians(1500.0); // rad/s
    };
    protected final static double kDt = 0.01; // s
    protected final static double kMaxSteeringVelocityError = Math.toRadians(2.0); // rad/s
    protected final static double kMaxAccelerationError = 0.1; // m/s^2

    public void SatisfiesConstraints(SwerveSetpoint prev, SwerveSetpoint next) {
        for (int i = 0; i < prev.mModuleStates.length; ++i) {
            final var prevModule = prev.mModuleStates[i];
            final var nextModule = next.mModuleStates[i];
            Rotation2d diffRotation = prevModule.angle.inverse().rotateBy(nextModule.angle);
            assertTrue(Math.abs(diffRotation.getRadians()) < kKinematicLimits.kMaxSteeringVelocity
                    + kMaxSteeringVelocityError);
            assertTrue(Math.abs(nextModule.speedMetersPerSecond) <= kKinematicLimits.kMaxDriveVelocity);
            assertTrue(Math.abs(nextModule.speedMetersPerSecond - prevModule.speedMetersPerSecond)
                    / kDt <= kKinematicLimits.kMaxDriveAcceleration + kMaxAccelerationError);
        }
    }

    public void CompareSetpoints(SwerveSetpoint prev, ChassisSpeeds goal, SwerveSetpointGenerator generator,
            SwerveSetpointGenerator2 generator2) {
        var next = generator.generateSetpoint(kKinematicLimits, prev, goal, kDt);
        var next2 = generator2.generateSetpoint(kKinematicLimits2, prev, goal, kDt);
        SatisfiesConstraints(prev, next);
        SatisfiesConstraints(prev, next2);
        assertEquals(0, next2.mChassisSpeeds.vxMetersPerSecond, 0.01);
        assertEquals(0, next2.mChassisSpeeds.omegaRadiansPerSecond, 0.01);
        assertEquals(0, next2.mChassisSpeeds.vyMetersPerSecond, 0.01);
        assertEquals(0, next.mChassisSpeeds.vxMetersPerSecond, 0.01);
        assertEquals(0, next.mChassisSpeeds.vyMetersPerSecond, 0.01);
        assertEquals(0, next.mChassisSpeeds.omegaRadiansPerSecond, 0.01);
        System.out.println(next);
        System.out.println(next2);

        assertTrue(Util.epsilonEquals(next.mChassisSpeeds.vxMetersPerSecond, next2.mChassisSpeeds.vxMetersPerSecond));
        assertTrue(Util.epsilonEquals(next.mChassisSpeeds.vyMetersPerSecond, next2.mChassisSpeeds.vyMetersPerSecond));
        assertTrue(Util.epsilonEquals(next.mChassisSpeeds.omegaRadiansPerSecond,
                next2.mChassisSpeeds.omegaRadiansPerSecond));
    }

    @Test
    public void testGenerateSetpoint() {
        SwerveModuleState[] initialStates = {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        };
        SwerveSetpoint setpoint = new SwerveSetpoint(new ChassisSpeeds(), initialStates);

        var generator = new SwerveSetpointGenerator(kKinematics);
        var generator2 = new SwerveSetpointGenerator2(kKinematics);

        var goalSpeeds = new ChassisSpeeds(3, 4, 3);
        CompareSetpoints(setpoint, goalSpeeds, generator, generator2);
    }
}