package frc.robot;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveAllAxes extends CommandBase {
  private final Supplier<Map<Arm.Axis, Double>> m_input;
  private final Arm m_arm;
  private static double kDt = 0.02;

  public MoveAllAxes(Supplier<Map<Arm.Axis, Double>> input, Arm output) {
    m_input = input;
    m_arm = output;
    addRequirements(output);
    SmartDashboard.putData("mover", this);
  }

  @Override
  public void initialize() {
    // Records the goals at the moment the command starts.
    m_arm.setGoals(m_input.get());
  }

  @Override
  public boolean isFinished() {
    return m_arm.atGoal();
  }

  @Override
  public void execute() {
    // while running, let the servos move towards their goals.
    m_arm.move(kDt);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty("isFinished", this::isFinished, null);
  }
}