package org.team100.frc2023.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.team100.frc2023.subsystems.AHRSClass;
import org.team100.frc2023.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveWithHeading extends CommandBase {
    private static final double kMaxSpeedM_S = 5;

    private final SwerveDriveSubsystem m_robotDrive;
    private final DoubleSupplier m_xSpeed1_1;
    private final DoubleSupplier m_ySpeed1_1;
    private final DoubleSupplier m_rotSpeed1_1;
    private final AHRSClass m_gyro;
    private final ProfiledPIDController m_headingController;
    private final Supplier<Rotation2d> m_desiredRotation;

    private Rotation2d lastRotationSetpoint;
    private boolean snapMode = false;
    private Pose2d currentPose;
    private double thetaOuput = 0;
    private double thetaControllerOutput;

    public DriveWithHeading(
            SwerveDriveSubsystem robotDrive,
            DoubleSupplier xSpeed,
            DoubleSupplier ySpeed,
            Supplier<Rotation2d> desiredRotation,
            DoubleSupplier rotSpeed,
            String name,
            AHRSClass gyro) {
        m_robotDrive = robotDrive;
        m_xSpeed1_1 = xSpeed;
        m_ySpeed1_1 = ySpeed;
        m_rotSpeed1_1 = rotSpeed;
        m_gyro = gyro;
        m_headingController = m_robotDrive.controllers.headingController;
        m_desiredRotation = desiredRotation;

        lastRotationSetpoint = new Rotation2d(0);

        m_headingController.enableContinuousInput(-Math.PI, Math.PI);
        currentPose = m_robotDrive.getPose();
        SmartDashboard.putData("Drive with Heading:", this);
        addRequirements(m_robotDrive);
    }

    @Override
    public void execute() {
        double xSpeed = MathUtil.applyDeadband(m_xSpeed1_1.getAsDouble(), .05);
        double ySpeed = MathUtil.applyDeadband(m_ySpeed1_1.getAsDouble(), .05);
        double rotSpeed = MathUtil.applyDeadband(m_rotSpeed1_1.getAsDouble(), .05);

        // Set desired rotation to POV (if pressed) or else last setpoint
        double desiredRotation = lastRotationSetpoint.getRadians();
        Rotation2d pov = m_desiredRotation.get();
        if (pov != null) {
            snapMode = true;
            desiredRotation = pov.getRadians();
        }
        currentPose = m_robotDrive.getPose();
        double currentRads = MathUtil.angleModulus(currentPose.getRotation().getRadians());

        if (snapMode && Math.abs(rotSpeed) < 0.1 && m_gyro.getGyroWorking()) {
            thetaControllerOutput = m_headingController.calculate(currentRads, desiredRotation);
            thetaOuput = thetaControllerOutput * kMaxSpeedM_S + m_headingController.getSetpoint().velocity;
        } else {
            snapMode = false;
            thetaOuput = (rotSpeed / 2) * kMaxSpeedM_S;
            desiredRotation = currentRads;
        }

        m_robotDrive.driveMetersPerSec(xSpeed * kMaxSpeedM_S, ySpeed * kMaxSpeedM_S, thetaOuput, true);
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
        builder.addDoubleProperty("Measurment", () -> currentPose.getRotation().getRadians(), null);
        builder.addDoubleProperty("Setpoint Position", () -> m_headingController.getSetpoint().position, null);
        builder.addDoubleProperty("Setpoint Velocity", () -> m_headingController.getSetpoint().velocity, null);
        builder.addDoubleProperty("Goal", () -> m_headingController.getGoal().position, null);
        builder.addDoubleProperty("HeadingControllerOutput", () -> thetaControllerOutput, null);
        builder.addDoubleProperty("Theta Outpit", () -> thetaOuput, null);
    }
}
