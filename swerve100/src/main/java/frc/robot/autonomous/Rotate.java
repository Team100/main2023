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
                m_robotDrive.thetaController,
                () -> m_robotDrive.getPose().getRotation().getRadians(),
                targetAngleRadians,
                (outPut, state) -> m_robotDrive.drive(0, 0, outPut, false),
                m_robotDrive);

        getController().enableContinuousInput(-Math.PI, Math.PI);
        getController().setTolerance(.1, .1);
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
        // boolean hasElapsed = m_timer.hasElapsed(3);
        // if (hasElapsed) {
        // System.out.println("BAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAALSLLSLSLSLSLSLS");
        // }
        // return hasElapsed;return

        // return m_haveMeasurement
        // && m_haveSetpoint
        // && Math.abs(m_positionError) < m_positionTolerance
        // && Math.abs(m_velocityError) < m_velocityTolerance;

        // boolean atSetPoint = m_controller.atSetpoint();
        // if (atSetPoint) {
        // System.out.println("LETS A FRICKING OGOOOOOOOOOOOOOOOOOOOOOO");
        // }
        // return atSetPoint && m_goal.get().equals(getController().getSetpoint());
        // return getController().atGoal();
        // && Math.abs(m_positionError) < m_positionTolerance
        // && Math.abs(m_velocityError) < m_velocityTolerance;

        double setpointMinusMeasurement = m_controller.getPositionError();
        double setpointMinusMeasurementVelocity = m_controller.getVelocityError();
        double positionTolerance = m_controller.getPositionTolerance();
        double velocityTolerance = m_controller.getVelocityTolerance();
        double goalPosition = m_goal.get().position;
        double goalVelocity = m_goal.get().velocity;
        double setpointPosition = getController().getSetpoint().position;
        double setpointVelocity = getController().getSetpoint().velocity;
        System.out.printf("%5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f\n",
                goalPosition,
                goalVelocity,
                setpointPosition,
                setpointVelocity,
                setpointMinusMeasurement,
                setpointMinusMeasurementVelocity,
                positionTolerance,
                velocityTolerance);
        double setpointMinusGoal = setpointVelocity - goalVelocity;
        double setpointMinusGoalPosition = setpointPosition - goalPosition;
        if (setpointMinusGoal < 0.1) {
            if (setpointMinusGoalPosition < 0.1) {
                if (setpointMinusMeasurement < 0.1) {
                    if (setpointMinusMeasurementVelocity < 0.1) {
                        return true;
                    }
                }
            }
        }
        // return (atSetPoint && Math.abs(m_goal.get() - getController().getSetpoint())
        // < 0.00001);
        return false;
    }

    @Override
    public void end(boolean isInterupted) {
        System.out.println("DONEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
        // m_timer.stop();
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
