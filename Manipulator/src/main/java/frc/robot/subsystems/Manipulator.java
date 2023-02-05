// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FRCLib.Motors.FRCTalonFX;
import frc.robot.FRCLib.Motors.FRCTalonSRX;
import frc.robot.FRCLib.Motors.FRCTalonSRX.FRCTalonSRXBuilder;

public class Manipulator extends SubsystemBase {
  /** Creates a new Manipulator. */
  FRCTalonSRX pinch;
  public Manipulator() {
    pinch = new FRCTalonSRXBuilder(1)
    // .withKP(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.KP)
    // .withKI(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.KI)
    // .withKD(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.KD)
    // .withKF(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.KF)
    .withInverted(false)
    // .withSensorPhase(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.SENSOR_PHASE)
    .withPeakOutputForward(0.5)
    .withPeakOutputReverse(-0.5)
    //.withNeutralMode(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.NEUTRAL_MODE)
    //.withCurrentLimitEnabled(true)
    //.withCurrentLimit(5)

    .build();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(pinch.getAppliedOutput());

  }

  public void pinch(double d){
    pinch.drivePercentOutput(d);

  }

  // public void initSendable(SendableBuilder builder) {
  //   super.initSendable(builder);
  //   builder.addDoubleProperty("Output", () -> );
  // }

}
