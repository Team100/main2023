package frc.robot.commands;

import java.util.Map;
import java.util.function.Supplier;
import java.util.HashMap;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Axis;

/** Reads goals from NetworkTables, supplies them as a map. */
public class GoalInput implements Sendable, Supplier<Map<Arm.Axis, Double>> {
  public final Map<Arm.Axis, Double> m_goals = new HashMap<Arm.Axis, Double>();

  public GoalInput() {
    for (Arm.Axis axis : Arm.Axis.values()) {
      m_goals.put(axis, 0.0);
    }
    SmartDashboard.putData("goal_reader", this);
  }

  @Override
  public Map<Axis, Double> get() {
    return m_goals;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("goal_input");
    for (Arm.Axis axis : m_goals.keySet()) {
      builder.addDoubleProperty(axis.name(), () -> m_goals.get(axis), (x) -> m_goals.put(axis, x));
    }
  }
}