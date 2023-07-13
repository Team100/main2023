package org.team100.frc2023.commands;

import java.util.function.DoubleSupplier;

import org.team100.frc2023.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Accepts [-1,1] input and scales it to the specified maximum speeds. */
public class DriveScaled extends CommandBase {
    private final DoubleSupplier xSpeed;
    private final DoubleSupplier ySpeed;
    private final DoubleSupplier rotSpeed;
    private final SwerveDriveSubsystem m_robotDrive;
    private final double kMaxSpeed;
    private final double kMaxRot;

    public DriveScaled(
            DoubleSupplier xSpeed,
            DoubleSupplier ySpeed,
            DoubleSupplier rotSpeed,
            SwerveDriveSubsystem robotDrive,
            double maxSpeedM_S,
            double maxRotSpeedRad_S) {
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotSpeed = rotSpeed;
        m_robotDrive = robotDrive;
        kMaxSpeed = maxSpeedM_S;
        kMaxRot = maxRotSpeedRad_S;
        addRequirements(m_robotDrive);
    }

    @Override
    public void execute() {
        m_robotDrive.driveScaled(
                xSpeed.getAsDouble(),
                ySpeed.getAsDouble(),
                rotSpeed.getAsDouble(),
                true,
                kMaxSpeed,
                kMaxRot);
    }
}
