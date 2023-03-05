package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.ArmController;

public class ManualArm extends CommandBase {
  /** Creates a new ManualArm. */
  ArmController arm;
  XboxController m_controller;
  public ManualArm(ArmController a, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = a;
    m_controller = controller;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.driveManually(m_controller.getLeftY(), m_controller.getRightX());
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
