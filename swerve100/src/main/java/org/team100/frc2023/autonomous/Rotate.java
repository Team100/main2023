package org.team100.frc2023.autonomous;

import org.team100.lib.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Rotate extends CommandBase {
    private final SwerveDriveSubsystem m_robotDrive;
    private final ProfiledPIDController m_controller;
    private State m_goal;

    public Rotate(SwerveDriveSubsystem drivetrain, double targetAngleRadians) {
        m_robotDrive = drivetrain;
        m_controller = drivetrain.controllers.rotateController;
        m_controller.enableContinuousInput(-Math.PI, Math.PI);
        m_controller.setTolerance(0.001);
        m_goal = new State(targetAngleRadians, 0);
        addRequirements(drivetrain);
        SmartDashboard.putData("ROTATE COMMAND", this);
    }

    @Override
    public void execute() {
        double output = m_controller.calculate(m_robotDrive.getPose().getRotation().getRadians(), m_goal);
        m_robotDrive.driveMetersPerSec(new Twist2d(0, 0, output), false);
    }

    @Override
    public void initialize() {
        m_controller.reset(m_robotDrive.getPose().getRotation().getRadians());
    }

    @Override
    public boolean isFinished() {
        return m_controller.atGoal();
    }

    @Override
    public void end(boolean isInterupted) {
        m_robotDrive.stop();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Error", () -> m_controller.getPositionError(), null);
        builder.addDoubleProperty("Measurment", () -> m_robotDrive.getPose().getRotation().getRadians(), null);
        builder.addDoubleProperty("Goal", () -> m_controller.getGoal().position, null);
        builder.addDoubleProperty("GoalVelocity", () -> m_controller.getGoal().velocity, null);
        builder.addDoubleProperty("Setpoint", () -> m_controller.getSetpoint().position, null);
    }
}
