package org.team100.frc2023.commands.Arm;

import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.team100.frc2023.subsystems.arm.ArmController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

// TODO: remove use of ProfiledPIDCommand
public class UpperArmToGoal extends ProfiledPIDCommand {
    private final ArmController m_arm;

    public static UpperArmToGoal factory(double position, ArmController arm, double velocity) {

        Supplier<State> goalSource = () -> new State(position, velocity);
        SimpleMotorFeedforward upperArmFeedforward = new SimpleMotorFeedforward(0.0, 0.3);

        DoubleSupplier positionRad = () -> arm.getUpperArm();
        // double outputPID;
        // measurement = positionRad;
        // final ArmFeedforward armFeedforward = new ArmFeedforward(
        // 0.0, // kS
        // 0.1, // kG
        // 0.0, // kV
        // 0.0 // kA
        // );

        BiConsumer<Double, State> useOutput = (output, setpoint) -> {
            System.out.println(output);
            // double feedforward = armFeedforward.calculate(setpoint.position,
            // setpoint.velocity);
            double newOutput = output + upperArmFeedforward.calculate(setpoint.velocity, 0);
            arm.upperArmSegment.setMotor(newOutput);

        };
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(2, 3);
        ProfiledPIDController controller = new ProfiledPIDController(2.2, 0, 0, constraints);
        controller.setTolerance(0.1); // radians\
        return new UpperArmToGoal(controller, positionRad, goalSource, useOutput, arm);
    }

    private UpperArmToGoal(
            ProfiledPIDController controller,
            DoubleSupplier measurementSource,
            Supplier<State> goalSource,
            BiConsumer<Double, State> useOutput,
            ArmController arm) {
        super(controller, measurementSource, goalSource, useOutput, arm.upperArmSegment);
        m_arm = arm;
        SmartDashboard.putData("Upper Arm To Goal", this);
    }

    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Setpoint", () -> getController().getSetpoint().position, null);
        builder.addDoubleProperty("Error", () -> getController().getPositionError(), null);
        builder.addDoubleProperty("Goal", () -> getController().getGoal().position, null);
        builder.addDoubleProperty("Measurement", () -> m_arm.getUpperArm(), null);
        builder.addBooleanProperty("AT GOAL", () -> getController().atGoal(), null);
    }

    @Override
    public boolean isFinished() {
        return getController().atGoal();
    }
}
