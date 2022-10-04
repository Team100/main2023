package frc.robot.subsystems;

import static java.util.Map.entry;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents the arm. Coordinates motion of multiple profiled servos. */
public class Arm extends SubsystemBase {
  public enum Axis {
    Swing, Boom, Stick, Wrist, Twist, Grip
  }

  public final Map<Axis, ProfiledServo> m_servos = Map.ofEntries(
      entry(Axis.Swing, new ProfiledServo("Swing", 0)),
      entry(Axis.Boom, new ProfiledServo("Boom", 1)),
      entry(Axis.Stick, new ProfiledServo("Stick", 2)),
      entry(Axis.Wrist, new ProfiledServo("Wrist", 3)),
      entry(Axis.Twist, new ProfiledServo("Twist", 4)),
      entry(Axis.Grip, new ProfiledServo("Grip", 5)));

  /*
   * Applies the specified goals, adjusting velocity/acceleration so that all the
   * axes will complete at the same time.
   * 
   * TODO: add boundary enforcement (e.g. raise boom to avoid hitting the table).
   */
  public void setGoals(Map<Arm.Axis, Double> goals) {
    double slowest_eta = slowestEta(goals);
    for (Axis axis : Axis.values()) {
      ProfiledServo pServo = m_servos.get(axis);
      if (goals.containsKey(axis)) {
        double goal = goals.get(axis);
        double eta = pServo.eta(goal);
        pServo.setGoal(goal, eta / slowest_eta);
      }
    }
  }

  /** Moves the servos. Should be called often. */
  public void move(double dt) {
    for (ProfiledServo pServo : m_servos.values()) {
      pServo.move(dt);
    }
  }

  public boolean atGoal() {
    for (ProfiledServo pServo : m_servos.values()) {
      if (!pServo.atGoal())
        return false;
    }
    return true;
  }

  /** Finds the slowest axis; use this to slow down the other axes. */
  private double slowestEta(Map<Arm.Axis, Double> goals) {
    double slowest_eta = 0;
    for (Axis axis : Axis.values()) {
      ProfiledServo pServo = m_servos.get(axis);
      if (goals.containsKey(axis)) {
        double goal = goals.get(axis);
        double eta = pServo.eta(goal);
        if (eta > slowest_eta) {
          slowest_eta = eta;
        }
      }
    }
    return slowest_eta;
  }
}