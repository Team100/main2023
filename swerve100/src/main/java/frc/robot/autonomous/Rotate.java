// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

/** Add your docs here. */
public class Rotate extends ProfiledPIDCommand {

    SwerveDriveSubsystem m_robotDrive;

    public Rotate(SwerveDriveSubsystem m_robotDrive, double targetAngleRadians) {
        super(
                // new PIDController(1, 0, 0),
                m_robotDrive.thetaController,
                () -> m_robotDrive.getPose().getRotation().getRadians(),
                targetAngleRadians,
                (outPut, state) -> m_robotDrive.drive(0, 0, outPut, false),
                m_robotDrive);

        getController().enableContinuousInput(-Math.PI, Math.PI);
        getController().setTolerance(.1, .1);
        this.m_robotDrive = m_robotDrive;
        
        SmartDashboard.putData("ROTATE COMMAND", this);

    }

    @Override
    public boolean isFinished() {
        // End when the controller is at the reference.
        return getController().atGoal();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Error", () -> getController().getPositionError(), null);
        builder.addDoubleProperty("Measurment", () -> this.m_robotDrive.getPose().getRotation().getRadians(), null);
        builder.addDoubleProperty("Position Error", () -> this.m_controller.getPositionError(), null);
        // builder.addDoubleProperty("Setpoint", () -> getController().getSetpoint(), null);

    }
}
