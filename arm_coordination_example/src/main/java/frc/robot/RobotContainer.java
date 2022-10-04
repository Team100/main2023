package frc.robot;

import java.util.Map;
import static java.util.Map.entry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmVisualization;
import frc.robot.commands.ConstantGoal;
import frc.robot.commands.GoalInput;
import frc.robot.commands.MoveAllAxes;
import frc.robot.subsystems.Arm;

/** Contains and binds components. */
public class RobotContainer {
  private final Arm m_arm = new Arm();
  private final GoalInput m_goalInput = new GoalInput();
  private final ArmVisualization m_visualizer = new ArmVisualization(m_arm);
  private final XboxController m_controller = new XboxController(0);
  private final Command m_mover = new MoveAllAxes(m_goalInput, m_arm);
  private final Command m_autoCommand = new PrintCommand("auto goes here later");

  public RobotContainer() {
    configureButtonBindings();
    m_visualizer.start();
  }

  private void configureButtonBindings() {
    // Moves to the spot specified in the network tables.
    new JoystickButton(m_controller, XboxController.Button.kA.value).whenHeld(m_mover);

    // Plays a sequence of preset moves.
    new JoystickButton(m_controller, XboxController.Button.kB.value).whenHeld(
        new SequentialCommandGroup(
            // boom straight up
            new MoveAllAxes(new ConstantGoal(Map.ofEntries(
                entry(Arm.Axis.Boom, 0.5))), m_arm),
            // stick horizontal
            new MoveAllAxes(new ConstantGoal(Map.ofEntries(
                entry(Arm.Axis.Stick, 0.5))), m_arm),
            // extend and touch the east table
            new MoveAllAxes(new ConstantGoal(Map.ofEntries(
                entry(Arm.Axis.Swing, 0.0),
                entry(Arm.Axis.Boom, 0.2),
                entry(Arm.Axis.Stick, 0.35),
                entry(Arm.Axis.Wrist, 0.5))), m_arm),
            // swing to the north and retract
            new MoveAllAxes(new ConstantGoal(Map.ofEntries(
                entry(Arm.Axis.Swing, 0.5),
                entry(Arm.Axis.Boom, 0.5),
                entry(Arm.Axis.Stick, 0.8),
                entry(Arm.Axis.Wrist, 0.2))), m_arm),
            // extend and touch the west table
            new MoveAllAxes(new ConstantGoal(Map.ofEntries(
                entry(Arm.Axis.Swing, 1.0),
                entry(Arm.Axis.Boom, 0.2),
                entry(Arm.Axis.Stick, 0.35),
                entry(Arm.Axis.Wrist, 0.15))), m_arm)));
  }

  public Command getAutonomousCommand() {
    return m_autoCommand;
  }
}