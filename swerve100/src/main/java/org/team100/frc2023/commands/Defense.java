package org.team100.frc2023.commands;

import org.team100.lib.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Defense extends CommandBase {
    private final SwerveDriveSubsystem m_robotDrive;

    public Defense(SwerveDriveSubsystem swerveDriveSubsystem) {
        m_robotDrive = swerveDriveSubsystem;
        addRequirements(swerveDriveSubsystem);
    }

    /**
     * Sets the wheels to make an "X" pattern
     */
    @Override
    public void execute() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = new SwerveModuleState(0, new Rotation2d(Math.PI / 4));
        states[1] = new SwerveModuleState(0, new Rotation2d(7 * Math.PI / 4));
        states[2] = new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4));
        states[3] = new SwerveModuleState(0, new Rotation2d(5 * Math.PI / 4));
        m_robotDrive.setModuleStates(states);
    }

}
