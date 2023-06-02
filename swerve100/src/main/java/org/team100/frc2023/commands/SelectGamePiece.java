package org.team100.frc2023.commands;

import org.team100.frc2023.subsystems.AutonGamePiece;
import org.team100.frc2023.subsystems.arm.ArmController;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SelectGamePiece extends CommandBase {
  /** Creates a new SelectGamePiece. */

  AutonGamePiece gamePiece;
  ArmController arm;
  public boolean done = false;
  public SelectGamePiece(AutonGamePiece gP, ArmController a) {
    // Use addRequirements() here to declare subsystem dependencies.
    gamePiece = gP;
    arm = a;
    // addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if(gamePiece == AutonGamePiece.CONE){
        arm.cubeMode = false;
    } else {
        arm.cubeMode = true;
    }

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
