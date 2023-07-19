package org.team100.frc2023.autonomous;

import org.team100.lib.motion.drivetrain.SpeedLimits;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystemInterface;
import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;

// replaced with our own versions
// import com.acmerobotics.roadrunner.profile.MotionProfile;
// import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
// import com.acmerobotics.roadrunner.profile.MotionState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Uses a timed profile, which avoids the bad ProfiledPidController behavior of
 * staying behind. Also demonstrates Roadrunner MotionProfiles.
 */
public class Rotate extends CommandBase {
    private final SwerveDriveSubsystemInterface m_robotDrive;
    private final SpeedLimits m_speedLimits;
    private final PIDController m_controller;
    private final Timer m_timer;
    private final MotionState m_goalState;
    MotionProfile profile; // set in initialize(), package private for testing
    MotionState reference; // updated in execute(), package private for testing.

    public Rotate(
            SwerveDriveSubsystemInterface drivetrain,
            SpeedLimits speedLimits,
            PIDController controller,
            Timer timer,
            double targetAngleRadians) {
        m_robotDrive = drivetrain;
        m_speedLimits = speedLimits;
        m_controller = controller;
        m_timer = timer;
        m_goalState = new MotionState(targetAngleRadians, 0);
        reference = new MotionState(0, 0);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // TODO: nonzero start velocity
        MotionState start = new MotionState(m_robotDrive.getPose().getRotation().getRadians(), 0);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start,
                m_goalState,
                m_speedLimits.kMaxAngularSpeedRadiansPerSecond,
                m_speedLimits.kMaxAngularAccelRad_SS,
                m_speedLimits.kMaxAngularJerkRadiansPerSecondCubed);
        m_timer.reset();
    }

    @Override
    public void execute() {
        double measurement = m_robotDrive.getPose().getRotation().getRadians();
        measurementPub.set(measurement);
        reference = profile.get(m_timer.get());
        refX.set(reference.getX());
        refV.set(reference.getV());
        double feedForward = reference.getV(); // this is radians/sec
        double controllerOutput = m_controller.calculate(measurement, reference.getX()); // radians
        error.set(m_controller.getPositionError());
        double totalOutput = feedForward + controllerOutput;
        m_robotDrive.driveInRobotCoords(new Twist2d(0, 0, totalOutput));
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() > profile.duration() && m_controller.atSetpoint();
    }

    @Override
    public void end(boolean isInterupted) {
        m_robotDrive.stop();
    }

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("rotate");
    private final DoublePublisher error = table.getDoubleTopic("error").publish();
    private final DoublePublisher measurementPub = table.getDoubleTopic("measurement").publish();
    private final DoublePublisher refX = table.getDoubleTopic("refX").publish();
    private final DoublePublisher refV = table.getDoubleTopic("refV").publish();

}
