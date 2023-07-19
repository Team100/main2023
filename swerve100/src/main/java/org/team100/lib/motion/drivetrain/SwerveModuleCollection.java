package org.team100.lib.motion.drivetrain;

import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

/** Represents the modules in the drivetrain. */
public class SwerveModuleCollection {
    private final SwerveModule m_frontLeft;
    private final SwerveModule m_frontRight;
    private final SwerveModule m_rearLeft;
    private final SwerveModule m_rearRight;

    public SwerveModuleCollection(
            SwerveModule frontLeft,
            SwerveModule frontRight,
            SwerveModule rearLeft,
            SwerveModule rearRight) {
        m_frontLeft = frontLeft;
        m_frontRight = frontRight;
        m_rearLeft = rearLeft;
        m_rearRight = rearRight;
    }

    public SwerveModulePosition[] positions() {
        return new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
        };
    }

    public void setDesiredStates(SwerveModuleState[] swerveModuleStates) {
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    void stop() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_rearLeft.stop();
        m_rearRight.stop();
    }

    /** Test and log. */
    void test(double[][] desiredOutputs, FileWriter writer) {
        m_frontLeft.test(desiredOutputs[0]);
        m_frontRight.test(desiredOutputs[1]);
        m_rearLeft.test(desiredOutputs[2]);
        m_rearRight.test(desiredOutputs[3]);
        try {
            if (writer != null) {
                writer.write("Timestamp: " + Timer.getFPGATimestamp() +
                        ", P" + m_frontLeft.getPosition().distanceMeters
                        + ", " + m_frontLeft.getState().speedMetersPerSecond + "\n");
                writer.flush();
            }
        } catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }
    }

    // TODO: do we need this?
    // public void resetEncoders() {
    //     m_frontLeft.resetDriveEncoders();
    //     m_frontRight.resetDriveEncoders();
    //     m_rearLeft.resetDriveEncoders();
    //     m_rearRight.resetDriveEncoders();
    // }
}
