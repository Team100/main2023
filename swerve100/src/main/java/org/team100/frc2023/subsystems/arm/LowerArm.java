package org.team100.frc2023.subsystems.arm;

import java.util.function.Supplier;

import org.team100.lib.motors.FRCNEO;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LowerArm extends SubsystemBase {
  /** Creates a new UpperArm. */
  FRCNEO lowerArmMotor;
  Supplier<Double> lowerArmAngleSupplier;

  public LowerArm(Supplier<Double> lowerArmAngleSupplier, FRCNEO lowerArmMotor) {
    this.lowerArmMotor = lowerArmMotor;
    this.lowerArmAngleSupplier = lowerArmAngleSupplier;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double x) {
    this.lowerArmMotor.motor.set(x);
  }

  public double getAngle() {
    return lowerArmAngleSupplier.get();
  }
}
