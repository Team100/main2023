package org.team100.frc2023.commands;

import java.util.function.Supplier;

import org.team100.lib.commands.DriveUtil;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

/** Accepts [-1,1] input and scales it to the specified maximum speeds. */
public class DriveScaled extends Command {
    private final Supplier<Twist2d> m_twistSupplier;
    private final SwerveDriveSubsystem m_robotDrive;
    private final Supplier<SpeedLimits> m_speedLimitsSupplier;

    
    // observers
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // current pose
    private final NetworkTable twist = inst.getTable("Drive Scaled Twist");
    private final DoublePublisher twistXPublisher = twist.getDoubleTopic("x").publish();
    private final DoublePublisher twistYPublisher = twist.getDoubleTopic("y").publish();
    private final DoublePublisher twistRotPublisher = twist.getDoubleTopic("theta").publish();
    

    public DriveScaled(
            Supplier<Twist2d> twistSupplier,
            SwerveDriveSubsystem robotDrive,
            Supplier<SpeedLimits> speedLimitsSupplier) {
        m_twistSupplier = twistSupplier;
        m_robotDrive = robotDrive;
        m_speedLimitsSupplier = speedLimitsSupplier;
        addRequirements(m_robotDrive);
    }

    
    @Override
    public void execute() {
        
        twistXPublisher.set(m_twistSupplier.get().dx);
        twistYPublisher.set(m_twistSupplier.get().dy);
        twistRotPublisher.set(m_twistSupplier.get().dtheta);

        Twist2d twistM_S = DriveUtil.scale(
            m_twistSupplier.get(),
            m_speedLimitsSupplier.get().speedM_S,
            m_speedLimitsSupplier.get().angleSpeedRad_S);

        Pose2d currentPose = m_robotDrive.getPose();
        SwerveState manualState = SwerveDriveSubsystem.incremental(currentPose, twistM_S);
        m_robotDrive.setDesiredState(manualState);

        System.out.println("AHJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJJ");
        
    }

    @Override
    public void end(boolean interrupted) {
        m_robotDrive.truncate();
    }
}
