// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team100.frc2023.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team100.frc2023.subsystems.SwerveDriveSubsystem;
import team100.frc2023.subsystems.Arm.ArmController;

public class SetConeMode extends CommandBase {
  /** Creates a new SetConeMode. */
  ArmController m_arm;
  SwerveDriveSubsystem m_robotDrive;

  boolean done;
  
  public SetConeMode(ArmController arm, SwerveDriveSubsystem robotDrive) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_arm = arm;
    m_robotDrive = robotDrive;

    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.cubeMode = false;
    m_robotDrive.visionDataProvider.indicator.cone();

    done = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
