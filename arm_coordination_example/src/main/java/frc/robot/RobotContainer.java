package frc.robot;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ManualMove;
import frc.robot.commands.MoveAllAxes;
import frc.robot.commands.MoveSequence;
import frc.robot.goal.NTGoal;
import frc.robot.subsystems.Arm;
import frc.robot.visualization.ArmVisualization;

/** Contains and binds components. */
public class RobotContainer {
  private final Arm m_arm = new Arm();
  private final ArmVisualization m_visualizer = new ArmVisualization(m_arm);
  private final Supplier<Map<Arm.Axis, Double>> m_ntGoal = new NTGoal();
  private final Command m_mover = new MoveAllAxes(m_ntGoal, m_arm);
  private final XboxController m_controller = new XboxController(0);
  private final Command m_manualMover = new ManualMove(m_controller, m_arm);
  private final Command m_moveSequence = new MoveSequence(m_arm);
  private final Command m_autoCommand = new PrintCommand("auto goes here later");

  public RobotContainer() {
    configureButtonBindings();
    m_arm.setDefaultCommand(m_manualMover);
    m_visualizer.start();
  }

  private void configureButtonBindings() {
    // Moves to the spot specified in the network tables.
    new JoystickButton(m_controller, XboxController.Button.kA.value).whenHeld(m_mover);

    // Plays a sequence of preset moves.
    new JoystickButton(m_controller, XboxController.Button.kB.value).whenHeld(m_moveSequence);
  }

  public Command getAutonomousCommand() {
    return m_autoCommand;
  }
}