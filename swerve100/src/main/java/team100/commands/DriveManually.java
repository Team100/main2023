package team100.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * Drives the drivetrain according to controller [-1,1] inputs.
 * Note the control inputs are conventionally oriented so the caller should
 * invert the controller signal if necessary.
 */
public class DriveManually extends CommandBase {
    // CONFIG
    private static final double kDeadband = .1;
    private static final boolean kFieldRelative = true;

    // INPUTS
    private final DoubleSupplier xSpeed;
    private final DoubleSupplier ySpeed;
    private final DoubleSupplier rotSpeed;

    // OUTPUT
    private final SwerveDriveSubsystem m_robotDrive;


    /**
     * @param xSpeed   forward-positive [-1, 1]
     * @param ySpeed   left-positive [-1, 1]
     * @param rotSpeed counterclockwise-positive [-1, 1]
     */
    public DriveManually(
            DoubleSupplier xSpeed,
            DoubleSupplier ySpeed,
            DoubleSupplier rotSpeed,
            SwerveDriveSubsystem drivetrain) {
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotSpeed = rotSpeed;
        m_robotDrive = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        m_robotDrive.drive(
            MathUtil.applyDeadband(xSpeed.getAsDouble(), kDeadband),
            MathUtil.applyDeadband(ySpeed.getAsDouble(), kDeadband),
            MathUtil.applyDeadband(rotSpeed.getAsDouble(), kDeadband),
            kFieldRelative
        );
    }
}
