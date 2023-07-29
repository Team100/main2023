package org.team100.frc2023.commands;

import java.util.function.Supplier;

import org.team100.lib.sensors.RedundantGyro;
import org.team100.lib.controller.DriveControllers;
import org.team100.lib.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveWithHeading extends CommandBase {
    private final Supplier<Twist2d> twistSupplier;
    private final SwerveDriveSubsystem m_robotDrive;
    private final double maxSpeedM_S;
    private final double maxRotSpeedRad_S;
    private final RedundantGyro m_gyro;
    private final ProfiledPIDController m_headingController;
    private final Supplier<Rotation2d> m_desiredRotation;

    private Rotation2d lastRotationSetpoint;
    private boolean snapMode = false;
    private Pose2d currentPose;
    private double thetaOuput = 0;
    private double thetaControllerOutput;

    /**
     * @param twistSupplier [-1,1]
     */
    public DriveWithHeading(
            Supplier<Twist2d> twistSupplier,
            SwerveDriveSubsystem robotDrive,
            DriveControllers controllers,
            double maxSpeedM_S,
            double maxRotSpeedRad_S,
            Supplier<Rotation2d> desiredRotation,
            RedundantGyro gyro) {
        this.twistSupplier = twistSupplier;
        m_robotDrive = robotDrive;
        this.maxSpeedM_S = maxSpeedM_S;
        this.maxRotSpeedRad_S = maxRotSpeedRad_S;
        m_gyro = gyro;
        m_headingController = controllers.headingController;
        m_desiredRotation = desiredRotation;

        lastRotationSetpoint = new Rotation2d(0);

        m_headingController.enableContinuousInput(-Math.PI, Math.PI);
        currentPose = m_robotDrive.getPose();
        SmartDashboard.putData("Drive with Heading:", this);
        addRequirements(m_robotDrive);
    }

    @Override
    public void execute() {
        // Set desired rotation to POV (if pressed) or else last setpoint
        double desiredRotation = lastRotationSetpoint.getRadians();
        Rotation2d pov = m_desiredRotation.get();
        if (pov != null) {
            snapMode = true;
            desiredRotation = pov.getRadians();
        }
        currentPose = m_robotDrive.getPose();
        double currentRads = MathUtil.angleModulus(currentPose.getRotation().getRadians());
        Twist2d twist = twistSupplier.get();

        if (snapMode && Math.abs(twist.dtheta) < 0.1 && m_gyro.getGyroWorking()) {
            thetaControllerOutput = m_headingController.calculate(currentRads, desiredRotation);
            thetaOuput = thetaControllerOutput * maxRotSpeedRad_S + m_headingController.getSetpoint().velocity;
        } else {
            snapMode = false;
            thetaOuput = twist.dtheta * maxRotSpeedRad_S;
            desiredRotation = currentRads;
        }
        Twist2d twistM_S = new Twist2d(twist.dx * maxSpeedM_S, twist.dy * maxSpeedM_S, thetaOuput);
        m_robotDrive.driveMetersPerSec(twistM_S, true);
        lastRotationSetpoint = new Rotation2d(desiredRotation);
    }

    @Override
    public void end(boolean interrupted) {
        snapMode = false;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Error", () -> m_headingController.getPositionError(), null);
        builder.addDoubleProperty("Measurement", () -> currentPose.getRotation().getRadians(), null);
        builder.addDoubleProperty("Setpoint Position", () -> m_headingController.getSetpoint().position, null);
        builder.addDoubleProperty("Setpoint Velocity", () -> m_headingController.getSetpoint().velocity, null);
        builder.addDoubleProperty("Goal Position", () -> m_headingController.getGoal().position, null);
        builder.addDoubleProperty("Goal Velocity", () -> m_headingController.getGoal().velocity, null);
        builder.addDoubleProperty("Controller Output", () -> thetaControllerOutput, null);
        builder.addDoubleProperty("Theta Outpit", () -> thetaOuput, null);
    }
}
