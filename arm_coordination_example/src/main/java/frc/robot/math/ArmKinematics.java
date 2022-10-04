package frc.robot.math;

import java.util.Map;

import frc.robot.subsystems.Arm;

public class ArmKinematics {

  public static final double kBoomLength = 20;
  public static final double kStickLength = 20;
  public static final double kWristLength = 5;

  // TODO: replace this with wpilib version when it's released
  public static class Translation3d {
    public final double m_x;
    public final double m_y;
    public final double m_z;

    public Translation3d() {
      this(0.0, 0.0, 0.0);
    }

    public Translation3d(double x, double y, double z) {
      m_x = x;
      m_y = y;
      m_z = z;
    }
  }

  public Translation3d forward(Map<Arm.Axis, Double> joints) {
    return null;
  }

  public Map<Arm.Axis, Double> inverse(Translation3d position) {
    return null;
  }
}