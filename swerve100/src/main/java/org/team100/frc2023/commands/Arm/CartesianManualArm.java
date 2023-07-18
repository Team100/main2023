package org.team100.frc2023.commands.Arm;

import java.util.function.Supplier;

import org.team100.frc2023.subsystems.arm.ArmSubsystem;
import org.team100.lib.subsystems.arm.ArmAngles;
import org.team100.lib.subsystems.arm.ArmKinematics;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Manual arm control in cartesian coordinates. */
public class CartesianManualArm extends CommandBase {
    private final ArmSubsystem m_arm;
    private final ArmKinematics m_armKinematics;
    private final Supplier<Double> m_dx;
    private final Supplier<Double> m_dy;

    /**
     * @param dx use Control.armX().
     * @param dy use Control.armY().
     */
    public CartesianManualArm(ArmSubsystem arm, ArmKinematics armKinematics, Supplier<Double> dx, Supplier<Double> dy) {
        m_arm = arm;
        m_armKinematics = armKinematics;
        m_dx = dx;
        m_dy = dy;
        addRequirements(arm);
    }

    /** Set a new reference equal to the current state plus the controller input. */
    @Override
    public void execute() {
        ArmAngles measurement = m_arm.getMeasurement();
        Translation2d current = m_armKinematics.forward(measurement);
        double xReference = current.getX() + m_dx.get();
        double yReference = current.getY() + m_dy.get();
        m_arm.setReference(m_armKinematics.inverse(new Translation2d(xReference, yReference)));
    }

    /** Set the reference equal to the current state. */
    @Override
    public void end(boolean interrupted) {
        m_arm.setReference(m_arm.getMeasurement());
    }
}
