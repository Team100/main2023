package frc.robot.goal;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.math.ArmKinematics;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Axis;

/** Accepts cartesian input, supplies joint positions. */
public class CartesianGoal implements Sendable, Supplier<Map<Arm.Axis, Double>> {
  private final ArmKinematics m_kinematics;
  private double m_x;
  private double m_y;
  private double m_z;

  public CartesianGoal(ArmKinematics kinematics) {
    m_kinematics = kinematics;
    SmartDashboard.putData("cartesian goal_reader", this);

  }

  @Override
  public Map<Axis, Double> get() {
    return m_kinematics.inverse(new ArmKinematics.Translation3d(m_x, m_y, m_z));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("cartesian goal_input");
    builder.addDoubleProperty("X", () -> m_x, (x) -> m_x = x);
    builder.addDoubleProperty("Y", () -> m_y, (y) -> m_y = y);
    builder.addDoubleProperty("Z", () -> m_z, (z) -> m_z = z);
  }
}
