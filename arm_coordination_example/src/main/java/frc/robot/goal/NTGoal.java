package frc.robot.goal;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Axis;

/** Reads goals from NetworkTables, supplies them as a map. */
public class NTGoal implements Sendable, Supplier<Map<Arm.Axis, Double>> {
  private final Map<Arm.Axis, Double> m_goals = new HashMap<Arm.Axis, Double>();

  public NTGoal() {
    m_goals.put(Arm.Axis.Swing, 0.0);
    m_goals.put(Arm.Axis.Boom, 0.0);
    m_goals.put(Arm.Axis.Stick, 0.0);
    m_goals.put(Arm.Axis.Wrist, 0.0);
    // m_goals.put(Arm.Axis.Twist, 0.0); // fixed
    // m_goals.put(Arm.Axis.Grip, 0.0); // manual
    SmartDashboard.putData("goal_reader", this);
  }

  @Override
  public Map<Axis, Double> get() {
    return Collections.unmodifiableMap(m_goals);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("goal_input");
    for (Arm.Axis axis : m_goals.keySet()) {
      builder.addDoubleProperty(axis.name(), () -> m_goals.get(axis), (x) -> m_goals.put(axis, x));
    }
  }
}