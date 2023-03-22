// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

/** Add your docs here. */
public class Rotate extends ProfiledPIDCommand {
    // private final Timer m_timer = new Timer();
    SwerveDriveSubsystem m_robotDrive;

    public Rotate(SwerveDriveSubsystem m_robotDrive, double targetAngleRadians) {
        super(
                // new PIDController(1, 0, 0),
                m_robotDrive.headingController,
                () -> m_robotDrive.getPose().getRotation().getRadians(),
                targetAngleRadians,
                (output, state) -> m_robotDrive.drive(0, 0, output, false),
                m_robotDrive);

        getController().enableContinuousInput(-Math.PI, Math.PI);
        getController().setTolerance(.05, .05);
        m_robotDrive.thetaController.atSetpoint();
        this.m_robotDrive = m_robotDrive;

        SmartDashboard.putData("ROTATE COMMAND", this);

    }

    // @Override
    // public void initialize() {
    // m_timer.start();
    // } 

    @Override
    public boolean isFinished() {
        return getController().atGoal();

        
    }

    @Override
    public void end(boolean isInterupted) {
        System.out.println("DONEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
        // m_timer.stop();

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
        // builder.addDoubleProperty("Position Error", () ->
        // this.m_controller.getPositionError(), null);
        // builder.addDoubleProperty("Setpoint", () -> getController().getSetpoint(),
        // null);

    }
}
