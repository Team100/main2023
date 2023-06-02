package org.team100.frc2023.subsystems.arm;

import java.util.function.Supplier;

import org.team100.lib.motors.FRCNEO;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UpperArm extends SubsystemBase {
  /** Creates a new UpperArm. */
  FRCNEO upperArmMotor;
  Supplier<Double> upperArmPositionSupplier;

  public UpperArm(Supplier<Double> upperArmPositionSupplier, FRCNEO upperArmMotor) {
    this.upperArmMotor = upperArmMotor;
    this.upperArmPositionSupplier = upperArmPositionSupplier;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double x) {
    this.upperArmMotor.motor.set(x);
  }

  public double getAngle() {
    return upperArmPositionSupplier.get();
  }
}
