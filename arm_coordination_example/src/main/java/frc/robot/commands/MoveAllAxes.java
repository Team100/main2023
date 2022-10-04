package frc.robot.commands;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;

/** Captures goals at initialization time, moves the arm. */
public class MoveAllAxes extends CommandBase {
  private final Supplier<Map<Arm.Axis, Double>> m_input;
  private final Arm m_arm;

  public MoveAllAxes(Supplier<Map<Arm.Axis, Double>> input, Arm output) {
    m_input = input;
    m_arm = output;
    addRequirements(m_arm);
    SmartDashboard.putData("move all axes", this);
  }

  /** Records the goals at the moment the command starts. */
  @Override
  public void initialize() {
    m_arm.setGoals(m_input.get());
  }

  @Override
  public boolean isFinished() {
    return m_arm.atGoal();
  }

  /** While running, lets the servos move towards their goals. */
  @Override
  public void execute() {
    m_arm.move(Robot.kDt);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty("isFinished", this::isFinished, null);
  }
}