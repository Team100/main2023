package org.team100.frc2023.autonomous;

import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;
import org.team100.lib.subsystems.SpeedLimits;
import org.team100.lib.subsystems.SwerveDriveSubsystemInterface;

// replaced with our own versions
// import com.acmerobotics.roadrunner.profile.MotionProfile;
// import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
// import com.acmerobotics.roadrunner.profile.MotionState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Uses a timed profile, which avoids the bad ProfiledPidController behavior of
 * staying behind.  Also demonstrates Roadrunner MotionProfiles.
 */
public class Rotate extends CommandBase {
    private final SwerveDriveSubsystemInterface m_robotDrive;
    private final SpeedLimits speedLimits;
    private final PIDController controller;
    private final Timer m_timer;
    private final MotionState goalState;
    MotionProfile profile; // set in initialize(), package private for testing
    MotionState reference; // updated in execute(), package private for testing.

    public Rotate(
            SwerveDriveSubsystemInterface drivetrain,
            SpeedLimits speedLimits,
            PIDController newController,
            Timer timer,
            double targetAngleRadians) {
        m_robotDrive = drivetrain;
        this.speedLimits = speedLimits;
        controller = newController;

        m_timer = timer;

        goalState = new MotionState(targetAngleRadians, 0);
        if (drivetrain != null)
            addRequirements(drivetrain);

        reference = new MotionState(0, 0);
        SmartDashboard.putData("ROTATE COMMAND", this);
    }

    @Override
    public void initialize() {
        // TODO: nonzero start velocity
        MotionState start = new MotionState(m_robotDrive.getPose().getRotation().getRadians(), 0);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start,
                goalState,
                speedLimits.kMaxAngularSpeedRadiansPerSecond,
                speedLimits.kMaxAngularAccelRad_SS,
                speedLimits.kMaxAngularJerkRadiansPerSecondCubed);
        m_timer.reset();
    }

    @Override
    public void execute() {
        double measurement = m_robotDrive.getPose().getRotation().getRadians();
        reference = profile.get(m_timer.get());
        double feedForward = reference.getV(); // this is radians/sec
        double controllerOutput = controller.calculate(measurement, reference.getX()); // radians
        double totalOutput = feedForward + controllerOutput;
        m_robotDrive.driveMetersPerSec(new Twist2d(0, 0, totalOutput), false);
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() > profile.duration() && controller.atSetpoint();
    }

    @Override
    public void end(boolean isInterupted) {
        m_robotDrive.stop();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Error", () -> controller.getPositionError(), null);
        builder.addDoubleProperty("Measurement", () -> m_robotDrive.getPose().getRotation().getRadians(), null);
        builder.addDoubleProperty("Reference Position", () -> reference.getX(), null);
        builder.addDoubleProperty("Reference Velocity", () -> reference.getV(), null);
        builder.addDoubleProperty("Setpoint", () -> controller.getSetpoint(), null);
    }
}
