// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.ArmController;

public class SetConeMode extends CommandBase {
  /** Creates a new SetConeMode. */
  ArmController m_arm;

  boolean done;
  
  public SetConeMode(ArmController arm) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_arm = arm;

    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.cubeMode = false;

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
    return false;
  }
}
