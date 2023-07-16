package org.team100.frc2023.commands.Arm;

import org.team100.frc2023.subsystems.arm.ArmController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LowerArmToGoal extends CommandBase {
    public static class Config {
        public TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(2, 3);
    }

    private final Config m_config = new Config();
    private final ArmController m_arm;
    private final ProfiledPIDController m_controller;
    private State m_goal;

    public LowerArmToGoal(double position, ArmController arm) {
        m_goal = new State(position, 0);
        m_arm = arm;
        m_controller = new ProfiledPIDController(1.2, 0, 0, m_config.constraints);
        m_controller.setTolerance(0.1); // radians
        addRequirements(arm);
        SmartDashboard.putData("Lower Arm To Goal", this);
    }

    @Override
    public void initialize() {
        m_controller.reset(m_arm.getLowerArm());
    }

    @Override
    public void execute() {
        m_arm.lowerArmSegment.setMotor(
                m_controller.calculate(m_arm.getLowerArm(), m_goal));
    }

    @Override
    public boolean isFinished() {
        return m_controller.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.lowerArmSegment.setMotor(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Setpoint", () -> m_controller.getSetpoint().position, null);
        builder.addDoubleProperty("Error", () -> m_controller.getPositionError(), null);
        builder.addDoubleProperty("Goal", () -> m_controller.getGoal().position, null);
        builder.addDoubleProperty("Measurement", () -> m_arm.getUpperArm(), null);
    }
}
