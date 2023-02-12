// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

/** Add your docs here. */
public class Rotate extends PIDCommand {

    public Rotate(SwerveDriveSubsystem m_robotDrive, double targetAngleDegrees) {
        super(
                new PIDController(1, 0, 0),
                () -> m_robotDrive.getHeading().getRadians(),
                targetAngleDegrees,
                outPut -> m_robotDrive.drive(0, 0, outPut, false),
                m_robotDrive);
        getController().enableContinuousInput(-Math.PI, Math.PI);
        getController().setTolerance(.1, .1);
    }

    @Override
    public boolean isFinished() {
        // End when the controller is at the reference.
        return getController().atSetpoint();
    }
}
