// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public abstract class TrajectoryCommand extends CommandBase {
    SwerveDriveSubsystem m_robotDrive;
    SwerveControllerCommand s;

    /** Creates a new TrajectoryCommand. */
    public TrajectoryCommand(SwerveDriveSubsystem m_robotDrive) {
        this.m_robotDrive = m_robotDrive;

        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(this.m_robotDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        s = new SwerveControllerCommand(
            genTrajectory(m_robotDrive),
            m_robotDrive::getPose,
            SwerveDriveSubsystem.kDriveKinematics,
            m_robotDrive.xController,
            m_robotDrive.yController,
            m_robotDrive.thetaController,
            () -> m_robotDrive.getPose().getRotation(),
            m_robotDrive::setModuleStates,
            m_robotDrive);

        // CommandScheduler.getInstance().schedule(
        //     s.finallyDo( (interrupted) -> { done = true; })
        // );

        CommandScheduler.getInstance().schedule(s);

        // CommandScheduler.getInstance().schedule(
        //     s.asProxy().finallyDo((interrupted) -> { done = true; })
        // );


    }

    @Override
    public void execute(){
        System.out.println("YOOOOOOOOOOO");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        if(s.isFinished()){
            System.out.println("FINISHED++++++++++++++++++++++++++++++++++++++++");
            return true;
        }

        // return s.isFinished();
        return false;
    }

    public abstract Trajectory genTrajectory(SwerveDriveSubsystem m_robotDrive);
}
