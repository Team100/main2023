// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team100.frc2023.commands.Retro;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team100.frc2023.Retro.Illuminator;

public class LedOn extends CommandBase {
  /** Creates a new LedOn. */

  Illuminator illuminator;
  public LedOn(Illuminator i) {
    // Use addRequirements() here to declare subsystem dependencies.
    illuminator = i;  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    illuminator.set(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    illuminator.set(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    illuminator.set(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
