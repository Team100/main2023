package org.team100.frc2023.commands;

import org.team100.lib.commands.DriveUtil;
import org.team100.lib.subsystems.RedundantGyro;
import org.team100.lib.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoLevel extends CommandBase {
    private static final double kMaxSpeed = 4.5;
    private static final double kMaxRot = 5;
    private static final double kCruiseSpeed = 1.5;
    /** max speed as a fraction */
    private static final double kSpeedClamp1_1 = 0.08;
    // TODO: is this unit correct?
    private static final double kSpeedPerDegree = 0.005;
    private final boolean m_reversed;
    private final SwerveDriveSubsystem m_robotDrive;
    private final RedundantGyro m_gyro;
    private int count;

    // TODO: what is "reversed" for?
    public AutoLevel(boolean reversed, SwerveDriveSubsystem robotDrive, RedundantGyro gyro) {
        m_reversed = reversed;
        m_robotDrive = robotDrive;
        m_gyro = gyro;
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize() {
        count = 0;
    }

    public void execute() {
        double Roll = m_gyro.getRedundantRoll();
        double Pitch = m_gyro.getRedundantPitch();
        double ySpeed = MathUtil.clamp(kSpeedPerDegree * Roll, -kSpeedClamp1_1, kSpeedClamp1_1);
        double xSpeed = MathUtil.clamp(kSpeedPerDegree * Pitch, -kSpeedClamp1_1, kSpeedClamp1_1);

        if (m_reversed) {
            if (Math.abs(Roll) > 2.5 || Math.abs(Pitch) > 2.5) {
                count = 0;

                Twist2d twist = new Twist2d(xSpeed, ySpeed, 0);
                Twist2d twistM_S = DriveUtil.scale(twist, kMaxSpeed, kMaxRot);
                m_robotDrive.driveMetersPerSec(twistM_S, false);
            } else {
                count++;
            }
        } else {
            if (m_robotDrive.getPose().getX() >= 3.277) {
                if (Math.abs(Roll) > 2.5 || Math.abs(Pitch) > 2.5) {
                    count = 0;

                    Twist2d twist = new Twist2d(xSpeed, -ySpeed, 0);
                    Twist2d twistM_S = DriveUtil.scale(twist, kMaxSpeed, kMaxRot);
                    m_robotDrive.driveMetersPerSec(twistM_S, false);
                } else {
                    count++;
                }
            } else {
                Twist2d twistM_S = new Twist2d(kCruiseSpeed, 0, 0);
                m_robotDrive.driveMetersPerSec(twistM_S, true);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return count >= 20;
    }
}
