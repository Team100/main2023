package team100.control;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.MoveToAprilTag;
//import frc.robot.autonomous.SanjanAutonomous;
import frc.robot.commands.ArmHigh;
import frc.robot.commands.ResetPose;

/**
 * see https://docs.google.com/document/d/1M89x_IiguQdY0VhQlOjqADMa6SYVp202TTuXZ1Ps280/edit#
 */
public class DualXboxControl implements Control {
    private final CommandXboxController controller0;
    private final CommandXboxController controller1;

    public DualXboxControl() {
        controller0 = new CommandXboxController(0);
        System.out.printf("Controller0: %s\n",
                controller0.getHID().getName());
        controller1 = new CommandXboxController(1);
        System.out.printf("Controller1: %s\n",
                controller1.getHID().getName());
    }

    @Override
    public void resetPose(ResetPose command) {
        // TODO: choose one
        controller0.leftBumper().onTrue(command);
        controller0.a().onTrue(command);
    }

	@Override
	public void moveToAprilTag(MoveToAprilTag command) {
		controller0.b().onTrue(command);
	}

    // TODO: decide what "Y" should do.

	@Override
	public void autoLevel(frc.robot.commands.autoLevel command) {
		controller0.y().onTrue(command);
	}

	// @Override
	// public void sanjanAuto(SanjanAutonomous command) {
	// 	controller0.y().onTrue(command);
	// }

    @Override
    public void armHigh(ArmHigh command) {
        controller1.b().onTrue(command);  
    }

}
