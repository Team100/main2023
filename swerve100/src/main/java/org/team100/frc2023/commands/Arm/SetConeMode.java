package org.team100.frc2023.commands.Arm;

import org.team100.frc2023.subsystems.SwerveDriveSubsystem;
import org.team100.frc2023.subsystems.arm.ArmController;

import edu.wpi.first.wpilibj2.command.CommandBase;

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
    m_robotDrive.indicator.yellow();

    done = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
