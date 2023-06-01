package team100.frc2023.commands.Arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team100.frc2023.subsystems.arm.ArmController;

public class ManualArm extends CommandBase {
    private final ArmController arm;
    private final XboxController m_controller;

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
