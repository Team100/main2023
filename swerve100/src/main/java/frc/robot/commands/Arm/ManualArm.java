package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.ArmController;

public class ManualArm extends CommandBase {
    private final ArmController arm;
    private final XboxController m_controller;

    // TODO: get rid of the controller argument, use x and y suppliers instead.
    public ManualArm(ArmController a, XboxController controller) {
        arm = a;
        m_controller = controller;
        addRequirements(arm);
    }

    @Override
    public void execute() {

        // if(arm.getLowerArm() <= arm.softStop && m_controller.getRightX()/4 < 0){
        //     arm.driveManually(m_controller.getLeftY()/2, 0);
        // } else {
            arm.driveManually(m_controller.getLeftY()/4, m_controller.getRightX()/4);

        // }
    }
}
