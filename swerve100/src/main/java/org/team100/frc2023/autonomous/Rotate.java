package org.team100.frc2023.autonomous;

import org.team100.frc2023.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

// TODO: do we need this?
public class Rotate extends ProfiledPIDCommand {
    private final SwerveDriveSubsystem m_robotDrive;

    public Rotate(SwerveDriveSubsystem drivetrain, double targetAngleRadians) {
        super(
                drivetrain.rotateController,
                () -> drivetrain.getPose().getRotation().getRadians(),
                targetAngleRadians,
                (output, state) -> drivetrain.drive(0, 0, output, false),
                drivetrain);
        getController().enableContinuousInput(-Math.PI, Math.PI);
        getController().setTolerance(0.001);
        drivetrain.thetaController.atSetpoint();
        m_robotDrive = drivetrain;
        SmartDashboard.putData("ROTATE COMMAND", this);
    }

    @Override
    public boolean isFinished() {
        return getController().atGoal();
    }

    @Override
    public void end(boolean isInterupted) {
        m_robotDrive.m_frontLeft.setOutput(0, 0);
        m_robotDrive.m_frontRight.setOutput(0, 0);
        m_robotDrive.m_rearLeft.setOutput(0, 0);
        m_robotDrive.m_rearRight.setOutput(0, 0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Error", () -> getController().getPositionError(), null);
        builder.addDoubleProperty("Measurment", () -> this.m_robotDrive.getPose().getRotation().getRadians(), null);
        builder.addDoubleProperty("Goal", () -> getController().getGoal().position, null);
        builder.addDoubleProperty("GoalVelocity", () -> getController().getGoal().velocity, null);
        builder.addDoubleProperty("Setpoint", () -> getController().getSetpoint().position, null);
    }
}
