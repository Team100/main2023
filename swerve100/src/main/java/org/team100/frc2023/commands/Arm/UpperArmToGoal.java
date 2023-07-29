package org.team100.frc2023.commands.Arm;

import org.team100.frc2023.subsystems.arm.ArmController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class UpperArmToGoal extends CommandBase {
    private static final SimpleMotorFeedforward upperArmFeedforward = new SimpleMotorFeedforward(0.0, 0.3);
    private static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(2, 3);
    private final ArmController m_arm;
    private final ProfiledPIDController m_controller;
    private State m_goal;

    public UpperArmToGoal(double position, ArmController arm, double velocity) {
        m_arm = arm;
        m_controller = new ProfiledPIDController(2.2, 0, 0, constraints);
        m_controller.setTolerance(0.1);
        m_goal = new State(position, velocity);
        addRequirements(arm);
        SmartDashboard.putData("Upper Arm To Goal", this);
    }

    @Override
    public void initialize() {
        m_controller.reset(m_arm.getUpperArm());
    }

    @Override
    public void execute() {
        m_arm.upperArmSegment.setMotor(
                m_controller.calculate(m_arm.getUpperArm(), m_goal)
                        + upperArmFeedforward.calculate(m_controller.getSetpoint().velocity, 0));

    }

    @Override
    public boolean isFinished() {
        return m_controller.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.upperArmSegment.setMotor(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Setpoint", () -> m_controller.getSetpoint().position, null);
        builder.addDoubleProperty("Error", () -> m_controller.getPositionError(), null);
        builder.addDoubleProperty("Goal", () -> m_controller.getGoal().position, null);
        builder.addDoubleProperty("Measurement", () -> m_arm.getUpperArm(), null);
        builder.addBooleanProperty("AT GOAL", () -> m_controller.atGoal(), null);
    }
}
