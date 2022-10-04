package frc.robot;

import java.util.Map;
import java.util.function.Supplier;
import frc.robot.Arm.Axis;

public class ConstantGoal implements Supplier<Map<Arm.Axis, Double>> {
  public final Map<Arm.Axis, Double> m_goals;

  /*
   * = Map.ofEntries(
   * entry(Arm.Axis.Swing, 0.0),
   * entry(Arm.Axis.Boom, 0.0),
   * entry(Arm.Axis.Stick, 0.0),
   * entry(Arm.Axis.Wrist, 0.0),
   * entry(Arm.Axis.Twist, 0.0),
   * entry(Arm.Axis.Grip, 0.0));
   */

  public ConstantGoal(Map<Arm.Axis, Double> goals) {
    m_goals = goals;
  }

  @Override
  public Map<Axis, Double> get() {
    return m_goals;
  }
}
