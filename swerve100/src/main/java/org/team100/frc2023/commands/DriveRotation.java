package org.team100.frc2023.commands;

import java.util.function.Supplier;

import org.team100.frc2023.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveRotation extends CommandBase {
   private final SwerveDriveSubsystem m_robotDrive;
   private final Supplier<Double> rotSpeed;

    public DriveRotation(SwerveDriveSubsystem robotDrive, Supplier<Double> rot) {
        m_robotDrive = robotDrive;
        rotSpeed = rot;
        addRequirements(m_robotDrive);
    }

    @Override
    public void execute() {
        double rot = rotSpeed.get();
        if (Math.abs(rot) <= 0.15) {
            rot = 0;
        }
        m_robotDrive.driveMetersPerSec(new Twist2d(0, 0, rot), true);
    }

    @Override
    public void end(boolean interrupted) {
        Translation2d endTranslation2d = new Translation2d(m_robotDrive.getPose().getX(),
                m_robotDrive.getPose().getY());
        Pose2d endPose = new Pose2d(endTranslation2d, new Rotation2d(0));
        m_robotDrive.resetPose(endPose);
    }

}
