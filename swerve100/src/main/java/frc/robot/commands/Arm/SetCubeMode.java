package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.ArmController;

public class SetCubeMode extends CommandBase {
    ArmController m_arm;
    boolean done;

    public SetCubeMode(ArmController arm) {
        m_arm = arm;
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        m_arm.cubeMode = true;
        done = true;
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
