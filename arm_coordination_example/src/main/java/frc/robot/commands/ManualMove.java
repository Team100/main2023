package frc.robot.commands;

import static java.util.Map.entry;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;

/** Moves the arm incrementally using the xbox controller. */
public class ManualMove extends CommandBase {
  private static double kScale = 0.1;
  private static double kDeadband = 0.05;

  private final XboxController m_controller;
  private final Arm m_arm;

  public final Map<Arm.Axis, Double> m_goals = new HashMap<>(Map.ofEntries(
      entry(Arm.Axis.Swing, 0.0),
      entry(Arm.Axis.Boom, 0.0),
      entry(Arm.Axis.Stick, 0.0),
      entry(Arm.Axis.Wrist, 0.0),
      // entry(Arm.Axis.Twist, 0.0), // fixed, leave it alone
      entry(Arm.Axis.Grip, 0.0)));

  public ManualMove(XboxController controller, Arm arm) {
    m_controller = controller;
    m_arm = arm;
    addRequirements(m_arm);
  }

  /** Adjusts the goals and moves one timestep. */
  @Override
  public void execute() {
    // suggested key bindings in comments; see simgui-ds.json
    setGoal(Arm.Axis.Swing, m_controller::getLeftX); // sim: AD
    setGoal(Arm.Axis.Boom, m_controller::getLeftY); // sim: WS
    setGoal(Arm.Axis.Stick, m_controller::getRightY); // sim: IK
    setGoal(Arm.Axis.Wrist, m_controller::getRightX); // sim: JL
    // setGoal(Arm.Axis.Twist, null); // no control for twist
    setGoal(Arm.Axis.Grip, () -> m_controller.getLeftTriggerAxis() // sim: F
        - m_controller.getRightTriggerAxis()); // sim: H

    m_arm.setGoals(m_goals);
    m_arm.move(Robot.kDt);
  }

  /** Reads the current position and increments it using the supplied input. */
  private void setGoal(Arm.Axis axis, DoubleSupplier input) {
    m_goals.put(axis, m_arm.m_servos.get(axis).getPosition()
        + MathUtil.applyDeadband(kScale * input.getAsDouble(), kDeadband));
  }
}