// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team100.frc2023.subsystems.Arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team100.frc2023.FRCLib.Motors.FRCNEO;

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
