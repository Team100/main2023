// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FRCLib.Motors.FRCNEO;

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
