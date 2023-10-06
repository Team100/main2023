package org.team100.lib.autonomous;

import org.team100.lib.controller.State100;
import org.team100.lib.motion.drivetrain.Heading;
import org.team100.lib.motion.drivetrain.HeadingInterface;
import org.team100.lib.motion.drivetrain.SpeedLimit;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SpeedLimitsFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** Stops the drivetrain. */
public class DriveStop extends Command {
    private final SwerveDriveSubsystem m_robotDrive;
    private final HeadingInterface m_heading;
    private final Timer timer;
    public DriveStop(SwerveDriveSubsystem robotDrive, HeadingInterface heading) {
        m_robotDrive = robotDrive;
        m_heading = heading;
        timer = new Timer();

        // Pose2d currentPose = m_robotDrive.getPose();
        // double currentRads = MathUtil.angleModulus(currentPose.getRotation().getRadians());


        // MotionState start = new MotionState(currentRads, m_heading.getHeadingRateNWU());

        // MotionState goal = new MotionState(MathUtil.angleModulus(0), 0);
        // SpeedLimits limit = SpeedLimitsFactory.getSpeedLimits(SpeedLimit.Medium);
        // m_profile = MotionProfileGenerator.generateSimpleMotionProfile(
        //             start,
        //             goal,
        //             limit.angleSpeedRad_S,
        //             limit.angleAccelRad_S2);
                    
        addRequirements(m_robotDrive);

    }
    
    public void initialize(){

        
        
        timer.start();
    }

    @Override
    public void execute() {

        // MotionState snapRef = m_profile.get(timer.get());

        // m_robotDrive.setDesiredState(
        //             new SwerveState(
        //                     new State100(0, 0, 0),
        //                     new State100(0, 0, 0),
        //                     new State100(snapRef.getX(), snapRef.getV(), snapRef.getA())));
        m_robotDrive.stop();

    }

    @Override
    public void end(boolean interrupted) {
        // m_robotDrive.truncate();
    }
}
