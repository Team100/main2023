// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.Arm.ArmController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LowerArmToGoal extends ProfiledPIDCommand {

  private final ArmController arm;

  /** Creates a new LowerArmToGoal. */

  public static LowerArmToGoal factory(double position, ArmController arm) {

    Supplier<State> goalSource = () -> new State(position, 0);

    DoubleSupplier positionRad = () -> arm.getLowerArm();
    // double outputPID;
    // measurement = positionRad;
    // TODO: something about mass here
    final ArmFeedforward armFeedforward = new ArmFeedforward(
        0.0, // kS
        0.1, // kG
        0.0, // kV
        0.0 // kA
    );
    BiConsumer<Double, State> useOutput = (output, setpoint) -> {
      System.out.println(output);
      // double feedforward = armFeedforward.calculate(setpoint.position,
      // setpoint.velocity);
      arm.lowerArmSegment.setMotor(output);
    };
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
        2, // velocity rad/s
        3 // accel rad/s^2
    );
    ProfiledPIDController controller = new ProfiledPIDController(1.2, 0, 0, constraints);
    controller.setTolerance(0.1); // radians\
    return new LowerArmToGoal(controller, positionRad, goalSource, useOutput, arm);
  }

  private LowerArmToGoal(
      ProfiledPIDController controller,
      DoubleSupplier measurementSource,
      Supplier<State> goalSource,
      BiConsumer<Double, State> useOutput,
      ArmController arm) {

    super(controller, measurementSource,
        goalSource, useOutput, arm.lowerArmSegment);

    this.arm = arm;
    SmartDashboard.putData("Lower Arm To Goal", this);

  }

  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Setpoint", () -> getController().getSetpoint().position, null);
    builder.addDoubleProperty("Error", () -> getController().getPositionError(), null);
    builder.addDoubleProperty("Goal", () -> getController().getGoal().position, null);
    builder.addDoubleProperty("Measurement", () -> arm.getUpperArm(), null);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
