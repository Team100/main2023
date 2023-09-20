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
    private final SpeedLimits m_speedLimits;
    private final Supplier<Boolean[]> m_moderateSupplier; 

    
    // observers
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // current pose
    private final NetworkTable twist = inst.getTable("Drive Scaled Twist");
    private final DoublePublisher twistXPublisher = twist.getDoubleTopic("x").publish();
    private final DoublePublisher twistYPublisher = twist.getDoubleTopic("y").publish();
    private final DoublePublisher twistRotPublisher = twist.getDoubleTopic("theta").publish();
    private final BooleanPublisher moderateSlow = twist.getBooleanTopic("Slow").publish();
    private final BooleanPublisher moderateMedium= twist.getBooleanTopic("Medium").publish();

    public DriveScaled(
            Supplier<Twist2d> twistSupplier,
            SwerveDriveSubsystem robotDrive,
            SpeedLimits speedLimits) {
        m_twistSupplier = twistSupplier;
        m_robotDrive = robotDrive;
        m_speedLimits = speedLimits;
        m_moderateSupplier = null;
        addRequirements(m_robotDrive);
    }

    public DriveScaled(
            Supplier<Twist2d> twistSupplier,
            SwerveDriveSubsystem robotDrive,
            SpeedLimits speedLimits,
            Supplier<Boolean[]> moderateSupplier) {
        m_twistSupplier = twistSupplier;
        m_robotDrive = robotDrive;
        m_speedLimits = speedLimits;
        m_moderateSupplier = moderateSupplier;

        addRequirements(m_robotDrive);
    }

    
    @Override
    public void execute() {
        
        twistXPublisher.set(m_twistSupplier.get().dx);
        twistYPublisher.set(m_twistSupplier.get().dy);
        twistRotPublisher.set(m_twistSupplier.get().dtheta);

        // Boolean[] moderateArr;
        // if(m_moderateSupplier != null){
        //     moderateArr = m_moderateSupplier.get();
        // } else {
        //     moderateArr = new Boolean[]{false, false};
        // }


        // moderateSlow.set(moderateArr[0]);
        // moderateMedium.set(moderateArr[1]);
        // //slow
        // if(moderateArr[0] == true && moderateArr[1] == false){
        //     Twist2d twistM_S = DriveUtil.scale(
        //         m_twistSupplier.get(),
        //         DriveModerate.getSlow().speedM_S,
        //         DriveModerate.getSlow().angleSpeedRad_S);

        //     Pose2d currentPose = m_robotDrive.getPose();
        //     SwerveState manualState = SwerveDriveSubsystem.incremental(currentPose, twistM_S);
        //     m_robotDrive.setDesiredState(manualState);
        // } else if(moderateArr[1] == true){
        //     //medium

        //     Twist2d twistM_S = DriveUtil.scale(
        //         m_twistSupplier.get(),
        //         DriveModerate.getMedium().speedM_S,
        //         DriveModerate.getMedium().angleSpeedRad_S);

        //     Pose2d currentPose = m_robotDrive.getPose();
        //     SwerveState manualState = SwerveDriveSubsystem.incremental(currentPose, twistM_S);
        //     m_robotDrive.setDesiredState(manualState);

        // } else {
            Twist2d twistM_S = DriveUtil.scale(
                m_twistSupplier.get(),
                m_speedLimits.speedM_S,
                m_speedLimits.angleSpeedRad_S);

            Pose2d currentPose = m_robotDrive.getPose();
            SwerveState manualState = SwerveDriveSubsystem.incremental(currentPose, twistM_S);
            m_robotDrive.setDesiredState(manualState);
        // }
        
    }

    @Override
    public void end(boolean interrupted) {
        m_robotDrive.truncate();
    }
}
