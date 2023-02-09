package team100.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;

/**
 * Operates the manipulator according to controller [-1,1] inputs.
 */
public class GripManually extends CommandBase {
    // INPUT
    private final DoubleSupplier openSpeed;
    private final DoubleSupplier closeSpeed;
    // OUTPUT
    private final Manipulator manipulator;

    /**
     * If openSpeed is positive, overrides closeSpeed.
     * 
     * @param openSpeed  open-positive [0,1]
     * @param closeSpeed close-positive [0,1]
     */
    public GripManually(
            DoubleSupplier openSpeed,
            DoubleSupplier closeSpeed,
            Manipulator manipulator) {
        this.openSpeed = openSpeed;
        this.closeSpeed = closeSpeed;
        this.manipulator = manipulator;
        addRequirements(manipulator);
    }

    @Override
    public void execute() {
        manipulator.pinchv2(
                openSpeed.getAsDouble(),
                closeSpeed.getAsDouble());
    }

}
