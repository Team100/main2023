// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team100.frc2023.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team100.frc2023.subsystems.SwerveDriveSubsystem;

public class Defense extends CommandBase {
    private SwerveDriveSubsystem m_robotDrive;

    /** Creates a new Defense. */
    public Defense(SwerveDriveSubsystem swerveDriveSubsystem) {
        this.m_robotDrive = swerveDriveSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerveDriveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        // set the wheels to make an "X" pattern
        states[0] = new SwerveModuleState(0, new Rotation2d(Math.PI / 4));
        states[1] = new SwerveModuleState(0, new Rotation2d(7 * Math.PI / 4));
        states[2] = new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4));
        states[3] = new SwerveModuleState(0, new Rotation2d(5 * Math.PI / 4));

        m_robotDrive.setModuleStates(states);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
