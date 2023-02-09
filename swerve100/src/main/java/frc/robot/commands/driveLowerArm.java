package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

/**
 * Moves the arm according to controller [-1,1] inputs.
 */
public class driveLowerArm extends CommandBase {
    // INPUTS
    private final DoubleSupplier lowerSpeed;
    private final DoubleSupplier upperSpeed;

    // OUTPUT
    private final Arm arm;

    /**
     * @param lowerSpeed forward-positive [-1,1]
     * @param upperSpeed up-positive [-1,1]
     */
    public driveLowerArm(
            DoubleSupplier lowerSpeed,
            DoubleSupplier upperSpeed,
            Arm arm) {
        this.lowerSpeed = lowerSpeed;
        this.upperSpeed = upperSpeed;
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setLowerArm(lowerSpeed.getAsDouble());
        arm.setUpperArm(upperSpeed.getAsDouble());
        // arm.setBoth(controller.getLeftY(), controller.getRightX());
    }
}
