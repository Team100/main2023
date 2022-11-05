package frc.robot.commands;

import static java.util.Map.entry;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;

/** Plays a sequence of preset moves. */
public class MoveSequence extends SequentialCommandGroup {
  public MoveSequence(Arm m_arm) {
    addCommands(
        // boom straight up
        new MoveAllAxes(() -> Map.ofEntries(
            entry(Arm.Axis.Boom, 0.5)), m_arm),

        // stick horizontal
        new MoveAllAxes(() -> Map.ofEntries(
            entry(Arm.Axis.Stick, 0.5)), m_arm),

        // extend and touch the east table
        new MoveAllAxes(() -> Map.ofEntries(
            entry(Arm.Axis.Swing, 0.0),
            entry(Arm.Axis.Boom, 0.2),
            entry(Arm.Axis.Stick, 0.35),
            entry(Arm.Axis.Wrist, 0.5)), m_arm),

        // swing to the north and retract
        new MoveAllAxes(() -> Map.ofEntries(
            entry(Arm.Axis.Swing, 0.5),
            entry(Arm.Axis.Boom, 0.5),
            entry(Arm.Axis.Stick, 0.8),
            entry(Arm.Axis.Wrist, 0.2)), m_arm),

        // extend and touch the west table
        new MoveAllAxes(() -> Map.ofEntries(
            entry(Arm.Axis.Swing, 1.0),
            entry(Arm.Axis.Boom, 0.2),
            entry(Arm.Axis.Stick, 0.35),
            entry(Arm.Axis.Wrist, 0.15)), m_arm));
  }
}
