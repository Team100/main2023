package org.team100.lib.motion.drivetrain;

import java.io.FileWriter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The swerve drive in local, or robot, reference frame. This class knows
 * nothing about the outside world, it just accepts chassis speeds.
 */
public class SwerveLocal {
    private final SpeedLimits m_speedLimits;
    private final SwerveDriveKinematics m_DriveKinematics;
    private final SwerveModuleCollection m_modules;

    public SwerveLocal(
            SpeedLimits speedLimits,
            SwerveDriveKinematics driveKinematics,
            SwerveModuleCollection modules) {
        m_speedLimits = speedLimits;
        m_DriveKinematics = driveKinematics;
        m_modules = modules;
    }

    /**
     * Drives the modules to produce the target chassis speed.
     * 
     * @param targetChassisSpeeds speeds in robot coordinates.
     */
    public void setChassisSpeeds(ChassisSpeeds targetChassisSpeeds) {
        desiredSpeedXPublisher.set(targetChassisSpeeds.vxMetersPerSecond);
        desiredSpeedYPublisher.set(targetChassisSpeeds.vyMetersPerSecond);
        desiredSpeedRotPublisher.set(targetChassisSpeeds.omegaRadiansPerSecond);
        SwerveModuleState[] targetModuleStates = m_DriveKinematics.toSwerveModuleStates(targetChassisSpeeds);
        setModuleStates(targetModuleStates);
    }

    /**
     * Sets the wheels to make an "X" pattern.
     * TODO: let the drivetrain decide to do this when it's stopped for awhile
     */
    public void defense() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = new SwerveModuleState(0, new Rotation2d(Math.PI / 4));
        states[1] = new SwerveModuleState(0, new Rotation2d(7 * Math.PI / 4));
        states[2] = new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4));
        states[3] = new SwerveModuleState(0, new Rotation2d(5 * Math.PI / 4));
        setModuleStates(states);
    }

    public SwerveModulePosition[] positions() {
        return m_modules.positions();
    }

    public void stop() {
        m_modules.stop();
    }

    void test(double[][] desiredOutputs, FileWriter writer) {
        m_modules.test(desiredOutputs, writer);
    }

    ///////////////////////////////////////////////////////////

    private void setModuleStates(SwerveModuleState[] targetModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(targetModuleStates, m_speedLimits.kMaxSpeedMetersPerSecond);
        publishImpliedChassisSpeeds(targetModuleStates);
        m_modules.setDesiredStates(targetModuleStates);
    }

    /**
     * Publishes chassis speeds implied by the module settings. The difference from
     * the desired speed might be caused by, for example, desaturation.
     */
    private void publishImpliedChassisSpeeds(SwerveModuleState[] actualModuleState) {
        ChassisSpeeds actualChassisSpeeds = m_DriveKinematics.toChassisSpeeds(
                actualModuleState[0],
                actualModuleState[1],
                actualModuleState[2],
                actualModuleState[3]);
        speedXPublisher.set(actualChassisSpeeds.vxMetersPerSecond);
        speedYPublisher.set(actualChassisSpeeds.vyMetersPerSecond);
        speedRotPublisher.set(actualChassisSpeeds.omegaRadiansPerSecond);
        movingPublisher.set(isMoving(actualChassisSpeeds));
    }

    private static boolean isMoving(ChassisSpeeds speeds) {
        return (speeds.vxMetersPerSecond >= 0.1
                || speeds.vyMetersPerSecond >= 0.1
                || speeds.omegaRadiansPerSecond >= 0.1);
    }

    // observers
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // desired speed
    private final NetworkTable desired = inst.getTable("desired speed");
    private final DoublePublisher desiredSpeedXPublisher = desired.getDoubleTopic("x").publish();
    private final DoublePublisher desiredSpeedYPublisher = desired.getDoubleTopic("y").publish();
    private final DoublePublisher desiredSpeedRotPublisher = desired.getDoubleTopic("theta").publish();

    // actual speed
    private final NetworkTable speed = inst.getTable("actual speed");
    private final DoublePublisher speedXPublisher = speed.getDoubleTopic("x").publish();
    private final DoublePublisher speedYPublisher = speed.getDoubleTopic("y").publish();
    private final DoublePublisher speedRotPublisher = speed.getDoubleTopic("theta").publish();
    private final BooleanPublisher movingPublisher = speed.getBooleanTopic("moving").publish();

}
