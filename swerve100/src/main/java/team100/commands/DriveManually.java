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
    private static final double kSpeedModifier = 1.0;
    // TODO: make speed adjustable on the fly
    // private static final double kSpeedModifier = 0.5
    private static final boolean fieldRelative = true;
    //private static final boolean fieldRelative = false;

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
                xSpeed.getAsDouble() * kSpeedModifier,
                ySpeed.getAsDouble() * kSpeedModifier,
                rotSpeed.getAsDouble() * kSpeedModifier,
                fieldRelative);
    }

    // this is Ishan's work from feb 9.
    //@Override
    public void execute2() {
        double xSwitch = MathUtil.applyDeadband(xSpeed.getAsDouble(), .1);
        double ySwitch = MathUtil.applyDeadband(ySpeed.getAsDouble(), .1);
        double rotSwitch = MathUtil.applyDeadband(rotSpeed.getAsDouble(), .1);
        m_robotDrive.drive(
                Math.signum(xSwitch) * kSpeedModifier,
                Math.signum(ySwitch) * kSpeedModifier,
                Math.signum(rotSwitch) * kSpeedModifier,
                fieldRelative);
    }
}
