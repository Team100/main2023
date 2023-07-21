package org.team100.frc2023.commands;

import java.util.function.Supplier;

import org.team100.lib.commands.DriveUtil;
import org.team100.lib.controller.State100;
import org.team100.lib.motion.drivetrain.HeadingInterface;
import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveWithHeading extends Command {
    private final Supplier<Twist2d> m_twistSupplier;
    private final SwerveDriveSubsystem m_robotDrive;
    private final HeadingInterface m_heading;
    private final SpeedLimits m_speedLimits;
    private final Timer m_timer;
    private final Supplier<Rotation2d> m_desiredRotation;

    private boolean snapMode = false;
    private MotionState m_goal;
    private MotionProfile m_profile;
    private MotionState m_ref;

    /**
     * @param twistSupplier [-1,1]
     */
    public DriveWithHeading(
            Supplier<Twist2d> twistSupplier,
            SwerveDriveSubsystem robotDrive,
            HeadingInterface heading,
            SpeedLimits speedLimits,
            Timer timer,
            Supplier<Rotation2d> desiredRotation) {
        m_twistSupplier = twistSupplier;
        m_robotDrive = robotDrive;
        m_heading = heading;
        m_speedLimits = speedLimits;
        m_timer = timer;
        m_desiredRotation = desiredRotation;
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize() {
        snapMode = false;
    }

    @Override
    public void execute() {
        Pose2d currentPose = m_robotDrive.getPose();
        Rotation2d pov = m_desiredRotation.get();
        double currentRads = MathUtil.angleModulus(currentPose.getRotation().getRadians());

        if (pov != null) {
            // if you touch the pov switch, we turn on snap mode and make a new profile
            snapMode = true;

            // the new profile starts where we are now
            MotionState start = new MotionState(currentRads, m_heading.getHeadingRateNWU());

            // the new goal is simply the pov rotation with zero velocity
            m_goal = new MotionState(MathUtil.angleModulus(pov.getRadians()), 0);

            // new profile obeys the speed limits
            m_profile = MotionProfileGenerator.generateSimpleMotionProfile(
                    start,
                    m_goal,
                    m_speedLimits.angleSpeedRad_S,
                    m_speedLimits.angleAccelRad_S2,
                    m_speedLimits.angleJerkRad_S3);
            m_timer.reset();
        }


        Twist2d twist1_1 = m_twistSupplier.get();
        // if you touch the rotational control, we turn off snap mode
        if (Math.abs(twist1_1.dtheta) < 0.1) {
            snapMode = false;
        }

        if (snapMode) {
            // in snap mode we take dx and dy from the user, and use the profile for dtheta.
            m_ref = m_profile.get(m_timer.get());

            // this is user input
            Twist2d twistM_S = DriveUtil.scale(twist1_1, m_speedLimits.speedM_S, m_speedLimits.angleSpeedRad_S);
            // the snap overrides the user input for omega.
            Twist2d twistWithSnapM_S = new Twist2d(twistM_S.dx, twistM_S.dy, m_ref.getV());
            Twist2d twistM = DriveUtil.scale(twistWithSnapM_S, 0.02, 0.02);

            Pose2d ref = currentPose.exp(twistM);

            // cartesian is manual, rotation is direct from the profile
            // TODO: something about acceleration here
            m_robotDrive.setDesiredState(
                    new SwerveState(
                            new State100(ref.getX(), twistM_S.dx, 0),
                            new State100(ref.getY(), twistM_S.dy, 0),
                            new State100(m_ref.getX(), m_ref.getV(), m_ref.getA())));

        } else {
            // if we're not in snap mode then it's just pure manual
            Twist2d twistM_S = DriveUtil.scale(twist1_1, m_speedLimits.speedM_S, m_speedLimits.angleSpeedRad_S);
            SwerveState manualState = SwerveDriveSubsystem.incremental(currentPose, twistM_S);
            m_robotDrive.setDesiredState(manualState);
        }

        // publish what we did
        double headingMeasurement = currentPose.getRotation().getRadians();
        double headingRate = m_heading.getHeadingRateNWU();
        refX.set(m_ref.getX());
        refV.set(m_ref.getV());
        measurementX.set(headingMeasurement);
        measurementV.set(headingRate);
        errorX.set(m_ref.getX() - headingMeasurement);
        errorV.set(m_ref.getV() - headingRate);
    }

    @Override
    public void end(boolean interrupted) {
        m_robotDrive.truncate();
    }

    //////////////////////////////////////////////////////

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("rotate");
    private final DoublePublisher errorX = table.getDoubleTopic("errorX").publish();
    private final DoublePublisher errorV = table.getDoubleTopic("errorV").publish();
    private final DoublePublisher measurementX = table.getDoubleTopic("measurementX").publish();
    private final DoublePublisher measurementV = table.getDoubleTopic("measurementV").publish();
    private final DoublePublisher refX = table.getDoubleTopic("refX").publish();
    private final DoublePublisher refV = table.getDoubleTopic("refV").publish();
}
