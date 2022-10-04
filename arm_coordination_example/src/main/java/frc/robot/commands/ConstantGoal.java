package frc.robot.commands;

import java.util.Map;
import java.util.function.Supplier;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Axis;

/** Supplies a constant map of goals. */
public class ConstantGoal implements Supplier<Map<Arm.Axis, Double>> {
  public final Map<Arm.Axis, Double> m_goals;

  public ConstantGoal(Map<Arm.Axis, Double> goals) {
    m_goals = goals;
  }

  @Override
  public Map<Axis, Double> get() {
    return m_goals;
  }
}